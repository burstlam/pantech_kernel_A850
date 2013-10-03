/*
** =========================================================================
** Copyright (c) 2010-2012  Immersion Corporation.  All rights reserved.
**                          Immersion Corporation Confidential and Proprietary
**
** File:
**     ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control haptic force output (PWM duty cycle, amp enable/disable), save IVT file, etc...
**     This SPI controls the TI DRV2665 piezo haptic driver (silicon rev. 1.2 and 1.4).
** =========================================================================
*/

//#warning ************ Compiling Nexus S GC3 SPI using DRV2665 ************

#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/io.h>

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** DRV2665
*/
#define GAIN_50 0x00
#define GAIN_100 0x01
#define GAIN_150 0x02
#define GAIN_200 0x03
#define TIMEOUT_5MS 0x00
#define TIMEOUT_10MS 0x01
#define TIMEOUT_15MS 0x02
#define TIMEOUT_20MS 0x03
#define TIMEOUT_BITSHIFT 0x02

#define DRV2665_GAIN GAIN_150
#define DRV2665_TIMEOUT (TIMEOUT_20MS << TIMEOUT_BITSHIFT)

/*
** i2c bus that it sits on 
*/
#ifdef TS5000_SAVING_PT10
#define DEVICE_BUS  19
#else
#define DEVICE_BUS  12
#endif

/*
** Address of our device 
*/
#define DEVICE_ADDR 0x59

/*
** Name of the DRV2665 board
*/
#define DRV2665_BOARD_NAME   "ts5000-i2c"//"ts2665"

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS       1

/*
** I2C FW info
*/
typedef struct _FWINFO
{
    char cMajorVersion;
    char cMinorVersion;
    char cInterfaceType;    /* 3 = I2C */
    char cAmpType[9];       /* 1 = differential amp */
    char cAmpUsage;         /* 1 = single boost, 2 = dual boost */
    char cClockSource;      /* 2 = external 18MHz clock */
} FWINFO;
FWINFO g_FWInfo;

/*
** I2C bus dev structures (copied from linux/drivers/i2c/busses/i2c-s3c2410.c)
** Immersion has added one field in it to keep track of the index of the byte, 
** in the message, that is sent to the device, so we could resend the remaining 
** samples after a NACK. 
*/
enum s3c24xx_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

enum s3c24xx_i2c_type {
	TYPE_S3C2410,
	TYPE_S3C2440,
};

struct s3c24xx_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	unsigned int		suspended:1;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	unsigned int		tx_setup;
	unsigned int		irq;

	enum s3c24xx_i2c_state	state;
	unsigned long		clkrate;

	void __iomem		*regs;
	struct clk		*clk;
	struct device		*dev;
	struct resource		*ioarea;
	struct i2c_adapter	adap;

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif

    // Begin Immersion changes
    unsigned int        write_msg_byte_ptr;
    // End Immersion changes
};

/*
** TS5000 play command
*/
#define PLAY_CMD_PIEZO      0x0B  /* FIFO data Entry  register address for DRV2665 */

/*
** Number of NAK resend attempts before aborting
*/
#define NAK_RESEND_ATTEMPT  3

/*
** Wait time before returing from ImmVibeSPI_ForceOut_SetSamples() if nacked data are resent.
*/
#define WAIT_TIME_AFTER_LAST_NACKED_PACKET_MS 5

/*
** Wait time before disabling amp or going to standby
*/
#define WAIT_TIME_BEFORE_DISABLE_AMP_MS  15
#define AMP_ENABLE_DISABLE_TIMEOUT_MS 15

/*
** FIFO full bit flag: LSB of the register 0x00.
*/
#define FIFO_FULL 0x01

/*
** Low Battery bit flag: Bit 3 of the register 0x00.
*/
#define LOW_BATTERY 0x08

/*
** Global variables
*/
static bool g_bAmpEnabled[NUM_ACTUATORS] = { false, };
static struct i2c_client* g_pTheClient = NULL;
static VibeInt8 g_i2cBuf[VIBE_OUTPUT_SAMPLE_SIZE+1];
static char g_szFWVersion[VIBE_MAX_DEVICE_NAME_LENGTH];
static struct workqueue_struct *g_workqueueStruct = 0;
struct semaphore g_hSemaphoreAmpDisableEnable;

static void AmpDisableWorkQueueHandler(struct work_struct *w);
DECLARE_DELAYED_WORK(g_AmpDisableHandler, AmpDisableWorkQueueHandler);

/*
** TS5000/I2C structs and forward declarations
*/
#ifndef TS5000_SAVING_PT10
static struct i2c_board_info info = {
  I2C_BOARD_INFO(DRV2665_BOARD_NAME, DEVICE_ADDR),
};
#endif

static int ts5000_suspend(struct device *dev);
static int ts5000_resume(struct device *dev);
static int ts5000_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int ts5000_remove(struct i2c_client* client);
static const struct i2c_device_id ts5000_id[] =
{
    {DRV2665_BOARD_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, ts5000_id);

static const struct dev_pm_ops ts5000_i2c_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ts5000_suspend, ts5000_resume)
};
static struct i2c_driver ts5000_driver =
{
    .probe = ts5000_probe,
    .remove = __devexit_p(ts5000_remove),
    .id_table = ts5000_id,
    .driver =
    {
        .name = DRV2665_BOARD_NAME,
		.owner = THIS_MODULE,
        .pm = &ts5000_i2c_pm_ops,
    },
};

/*
** If standbyState is TRUE put the chip into standby mode (low-power state)
*/
static void setStandby(bool standbyState)
{
    struct i2c_msg msg[1];

    g_i2cBuf[0] = 0x02;
    msg[0].addr = g_pTheClient->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = g_i2cBuf;
    i2c_transfer(g_pTheClient->adapter, msg, 1);

    msg[0].flags = I2C_M_RD;
    msg[0].len = 1;
    msg[0].buf = g_i2cBuf+1;
    i2c_transfer(g_pTheClient->adapter, msg, 1);

    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = g_i2cBuf;
    if (standbyState && ((g_i2cBuf[1] & 0x40) == 0))
    {
        g_i2cBuf[1] = g_i2cBuf[1] | 0x40;
        i2c_transfer(g_pTheClient->adapter, msg, 1);
    } else if (!standbyState && ((g_i2cBuf[1] & 0x40) == 0x40))
    {
        g_i2cBuf[0] = 0x02;
        g_i2cBuf[1] = DRV2665_TIMEOUT; /* make sure chip is out of standby and set timeout to 20ms */

        msg[0].addr = g_pTheClient->addr;
        msg[0].flags = 0;//for write operation in register 2
        msg[0].len = 2;
        msg[0].buf = g_i2cBuf;

        i2c_transfer(g_pTheClient->adapter, msg, 1);
        
        g_i2cBuf[0] = 0x01;
        g_i2cBuf[1] = DRV2665_GAIN;
        msg[0].addr = g_pTheClient->addr;
        msg[0].flags = 0;//for write operation in register 1
        msg[0].len = 2;
        msg[0].buf = g_i2cBuf;

       i2c_transfer(g_pTheClient->adapter, msg, 1);
    }
}
/*
** Workqueue thread handler used to wait for WAIT_TIME_BEFORE_DISABLE_AMP_MS  before turning off the amp every time
** ImmVibeSPI_ForceOut_AmpDisable() is called, so the DRV2665 would have time to send all of the 
** samples that were buffered. Workqueue thread is used here as we don't want to block the call to 
** ImmVibeSPI_ForceOut_AmpDisable(). We cannot use a pure timer/interrupt solution here as the 
** setsandby() that is called here uses interrupts and could wait for mutex.
 */
static void AmpDisableWorkQueueHandler(struct work_struct *w)
{
    if (0 != down_timeout(&g_hSemaphoreAmpDisableEnable, msecs_to_jiffies(AMP_ENABLE_DISABLE_TIMEOUT_MS)))
    {
        DbgOut("AmpDisableWorkQueueHandler: down_timeout timed out.\n");
        return;
    }

    if (!g_bAmpEnabled[0]){  /* Don't go to standby mode if a new AmpEnable call was received during the sleep time above */
        setStandby(true);
    }
    up(&g_hSemaphoreAmpDisableEnable);
}

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    DbgOut("try to disable amp\n");
    if (0 != down_timeout(&g_hSemaphoreAmpDisableEnable, msecs_to_jiffies(AMP_ENABLE_DISABLE_TIMEOUT_MS)))
    {
        DbgOut("ImmVibeSPI_ForceOut_AmpDisable: down_timeout timed out.\n");
        return VIBE_E_FAIL;
    }

    if (g_bAmpEnabled[nActuatorIndex])
    {
        g_bAmpEnabled[nActuatorIndex] = false;

        /* 
        ** DRV2665 has standby bit to put it in low power mode 
        ** Wake up the the AmpDisable thread
        */
        if (g_workqueueStruct)
        {
            queue_delayed_work(g_workqueueStruct,
                               &g_AmpDisableHandler,
                               msecs_to_jiffies(WAIT_TIME_BEFORE_DISABLE_AMP_MS));
        }
    }
    up(&g_hSemaphoreAmpDisableEnable);
    DbgOut("Disabled amp\n");
    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    DbgOut("try to eanble amp\n");
    cancel_delayed_work_sync(&g_AmpDisableHandler);

    if (0 != down_timeout(&g_hSemaphoreAmpDisableEnable, msecs_to_jiffies(AMP_ENABLE_DISABLE_TIMEOUT_MS)))
    {
        DbgOut("ImmVibeSPI_ForceOut_AmpEnable: down_timeout timed out.\n");
        return VIBE_E_FAIL;
    }

    if (!g_bAmpEnabled[nActuatorIndex])
    {
        g_bAmpEnabled[nActuatorIndex] = true;

        /* DRV2665 has standby bit that needs to be cleared to enable the amp output  */
        if (0 == nActuatorIndex)
            setStandby(false);
    }
    up(&g_hSemaphoreAmpDisableEnable);
    DbgOut("Enabled amp\n");
    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    struct i2c_adapter* adapter; /* Initialized below. */
#ifndef TS5000_SAVING_PT10
    struct i2c_client* client; /* Initialized below. */
#endif
    struct i2c_msg msg[2];  /* initialized below */
    int nActuatorIndex;  /* Initialized below. */
#ifdef TS5000_SAVING_PT10
    int retVal = 0;
#endif
    DbgOut("ImmVibeSPI_ForceOut_Initialize.\n");

    adapter = i2c_get_adapter(DEVICE_BUS);
    if (adapter) {
        DbgOut("found device bus %d\n",DEVICE_BUS);
#ifndef TS5000_SAVING_PT10
        client = i2c_new_device(adapter, &info);
        if (client) {
            int retVal = i2c_add_driver(&ts5000_driver);
            if (retVal) {
                DbgOut("ts5000_driver: Cannot add driver.\n");
                return VIBE_E_FAIL;
            }
            DbgOut("added driver. retVal=%d addr=0x%02x name=%s\n",retVal, client->addr, client->name);
        } else {
            DbgOut("ts5000_driver: Cannot create new device.\n");
            return VIBE_E_FAIL;
        }
#else
        retVal = i2c_add_driver(&ts5000_driver);
        if (retVal) {
            DbgOut("ts5000_driver: Cannot add driver.\n");
            return VIBE_E_FAIL;
        }
        if(g_pTheClient == NULL) {
            DbgOut("i2c_client is null\n");
            return VIBE_E_FAIL;
        }
        DbgOut("added driver. retVal=%d addr=0x%02x name=%s\n",retVal, g_pTheClient->addr, g_pTheClient->name);
#endif
    } else {
        DbgOut("ts5000_driver: I2C Adapter not found.\n");
        return VIBE_E_FAIL;
    }

    /* To carry out  any register write, we need to enable  chip by clearing standby  bit */
    g_i2cBuf[0] = 0x02;
    g_i2cBuf[1] = DRV2665_TIMEOUT ; /* make sure chip is out of standby and set timeout to 20ms */

    msg[0].addr = g_pTheClient->addr;
    msg[0].flags = 0;//for write operation in register 2
    msg[0].len = 2;
    msg[0].buf = g_i2cBuf;
    i2c_transfer(g_pTheClient->adapter, msg, 1);

    /*
    Following  code is for setting gain for DRV2665  in register 1 bit 0 and bit 1
    0x00 :output  is set for +/-25V :For AAc, this is more useful  for +/-30V actuator
    0x01 :output  is set for +/-50V
    0x02 :output  is set for +/-75V :we use this for SEMCO
    0x03 :output  is set for +/-100V
    */
    g_i2cBuf[0] = 0x01;
    g_i2cBuf[1] = DRV2665_GAIN ;

    msg[0].addr = g_pTheClient->addr;
    msg[0].flags = 0;//for write operation in register 1
    msg[0].len = 2;
    msg[0].buf = g_i2cBuf;

    i2c_transfer(g_pTheClient->adapter, msg, 1);
    /* gain setting ends here */

    sema_init(&g_hSemaphoreAmpDisableEnable, 1); /* initialize semaphore that synchronize Amp disanling/enabling functions */
    g_workqueueStruct = create_workqueue("tspdrv_disable_amp_workqueue"); /* create workqueue to handle amp disabling thread */

    /* For each actuator... */
    for (nActuatorIndex = 0; NUM_ACTUATORS > nActuatorIndex; ++nActuatorIndex)
    {
        /* Disable amp */
        g_bAmpEnabled[nActuatorIndex] = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
        ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    int nActuatorIndex;  /* Initialized below. */

    DbgOut("ImmVibeSPI_ForceOut_Terminate.\n");

    if (g_workqueueStruct)
    {
        destroy_workqueue(g_workqueueStruct);
        g_workqueueStruct = 0;
    }

    /* For each actuator... */
    for (nActuatorIndex = 0; NUM_ACTUATORS > nActuatorIndex; ++nActuatorIndex)
    {
        /* Disable amp */
        ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);
    }

    /* Remove TS5000 driver */
    i2c_del_driver(&ts5000_driver);

    /* Reverse i2c_new_device */
    i2c_unregister_device(g_pTheClient);

    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    if (nActuatorIndex != 0) return VIBE_S_SUCCESS;

    switch (nFrequencyParameterID)
    {
        case VIBE_KP_CFG_FREQUENCY_PARAM1:
            /* Update frequency parameter 1 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM2:
            /* Update frequency parameter 2 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM3:
            /* Update frequency parameter 3 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM4:
            /* Update frequency parameter 4 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM5:
            /* Update frequency parameter 5 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM6:
          /* Update frequency parameter 6 */
            break;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    int txRes;              /* initialized below */
    struct i2c_msg msg[1];  /* initialized below */
    int i;                  /* initialized below */
    int nResendAttempt = NAK_RESEND_ATTEMPT;
    VibeStatus status = VIBE_E_FAIL;
    int nDataIndex = 0;
    int waitDurationMs = 0;

    if (!g_pTheClient) return VIBE_E_FAIL;
    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE) return VIBE_E_FAIL;
    if (nActuatorIndex > 0) return VIBE_E_FAIL;
    while (nResendAttempt--)
    {
        struct s3c24xx_i2c *i2c;
        int lastWrittenByteIndex;

        g_i2cBuf[0] = PLAY_CMD_PIEZO;

        msg[0].addr = g_pTheClient->addr;
        msg[0].flags = 0;

        for (i=0; i< (nBufferSizeInBytes - nDataIndex); ++i)
        {
            /* Copy the remaining data */
            g_i2cBuf[i+1] = pForceOutputBuffer[nDataIndex + i];
        }

        msg[0].len = nBufferSizeInBytes - nDataIndex + 1 /* room for PLAY_CMD */;
        msg[0].buf = g_i2cBuf;

        if ((txRes = i2c_transfer(g_pTheClient->adapter, msg, 1)) >= 0)
        {
            return VIBE_S_SUCCESS;
        }
        
        /* Save the index of the nacked byte */
        i2c = (struct s3c24xx_i2c *)g_pTheClient->adapter->algo_data;
        lastWrittenByteIndex = i2c->write_msg_byte_ptr;

        /* 
        ** Return VIBE_E_FAIL if i2c_transfer failure was caused by other reason than full fifo, otherwise,
        ** try to resend the remaining data.
        */
        g_i2cBuf[0] = 0x00;
        msg[0].addr = g_pTheClient->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = g_i2cBuf;

        msg[1].addr = g_pTheClient->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = g_i2cBuf+1;
        i2c_transfer(g_pTheClient->adapter, msg, 2);

        if (!(g_i2cBuf[1] & FIFO_FULL))
        {
            if (g_i2cBuf[1] & LOW_BATTERY)
                DbgOut("ImmVibeSPI_ForceOut_SetSamples failed due to low battery\n");

            return VIBE_E_FAIL;
        }

        if (0 < lastWrittenByteIndex) /* If the command byte (0x0B) was NACKed, do not change nDataIndex */
            nDataIndex += lastWrittenByteIndex - 1; /* The PLAY_CMB byte was counted by the I2C driver */

        /* Estimate wait duration time in ms before resending the unwritten bytes */
        /* DRV2665 FIFO read rate is 8 kHz - 125 us */
        /* Minimal calculated wait duration is 1 ms */
        waitDurationMs = 1 + ((nBufferSizeInBytes - nDataIndex) + 1) / 8;

        schedule_timeout_interruptible(msecs_to_jiffies(waitDurationMs));
    }

    return status;
}

/*
** Called to save an IVT data file (pIVT) to a file (szPathName)
*/
/* Not used by the TSP Kernel Module
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Save(const VibeUInt8 *pIVT, VibeUInt32 nIVTSize, const char *szPathname)
{
    DbgOut("ImmVibeSPI_IVTFile_Save.\n");

    return VIBE_S_SUCCESS;
}
*/

/*
** Called to delete an IVT file
*/
/* Not used by the TSP Kernel Module
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Delete(const char *szPathname)
{
    DbgOut("ImmVibeSPI_IVTFile_Delete.\n");

    return VIBE_S_SUCCESS;
}
*/

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    /* (d) == differential drive (-150V, +150V) */
    char szDeviceName[VIBE_MAX_DEVICE_NAME_LENGTH] = "DRV2665-EVK";

    DbgOut("ImmVibeSPI_Device_GetName.\n");

    if ((strlen(szDeviceName) + 1 + strlen(g_szFWVersion)) >= nSize)
        return VIBE_E_FAIL;

    sprintf(szDevName, "%s %s", szDeviceName, g_szFWVersion);

    return VIBE_S_SUCCESS;
}

/*
** TS5000 callback functions
*/
static int __devinit ts5000_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    int nRet = 0;
    char buf[1] = {0};

    DbgOut("ts5000_probe start\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        nRet = -ENODEV;
        DbgOut("ts5000_probe: i2c_check_functionality failed.\n");
    }
    else
    {
        g_pTheClient = client;
        DbgOut("addr=0x%02x name=%s\n",g_pTheClient->addr, g_pTheClient->name);

        /* Retrieve the FW version for information purposes */
        DbgOut("send to master, buf=%s\n",buf);
        i2c_master_send(g_pTheClient, (char*)(&buf), 1);
        DbgOut("recv from master, buf=%s\n",buf);
        if ((!g_pTheClient) || (i2c_master_recv(g_pTheClient, (char*)(&g_FWInfo), sizeof(FWINFO)) != sizeof(FWINFO)))
        {
            /* Could not retrieve the FW info. Writing ? as an indication */
            DbgOut("Could not retrieve the FW info. Writing ? as an indication\n");
            sprintf(g_szFWVersion, "[FW: ?]");
        }
        else
        {
            sprintf(g_szFWVersion, "[FW: v%d.%d]", g_FWInfo.cMajorVersion, g_FWInfo.cMinorVersion);
        }
    }
    DbgOut("ts5000_probe end. %s\n",g_szFWVersion);

    return nRet;
}

static int ts5000_remove(struct i2c_client *client)
{
    DbgOut("ts5000_remove.\n");

    g_pTheClient = NULL;

    return 0;
}

static int ts5000_suspend(struct device *dev)
{
    DbgOut("ts5000_suspend\n");
    return 0;
}

static int ts5000_resume(struct device *dev)
{
    DbgOut("ts5000_resume\n");
    return 0;
}
