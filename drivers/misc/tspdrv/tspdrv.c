/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description: 
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2011 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <tspdrv.h>
#include <linux/delay.h>

static int g_nTimerPeriodMs = 5; /* 5ms timer by default. This variable could be used by the SPI.*/

#include <ImmVibeSPI.c>
#if defined(VIBE_DEBUG) && defined(VIBE_RECORD)
#include <tspdrvRecorder.c>
#endif

/* Device name and version information */
#define VERSION_STR " v3.5.14.0\n"                  /* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[  (VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN)
                            * NUM_ACTUATORS];       /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying = false;

/* Buffer to store data sent to SPI */
#define MAX_SPI_BUFFER_SIZE (NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))

static char g_cWriteBuffer[MAX_SPI_BUFFER_SIZE];


#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

#ifndef HAVE_UNLOCKED_IOCTL
#define HAVE_UNLOCKED_IOCTL 0
#endif

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nMajor = 0;
#endif

/* Needs to be included after the global variables because they use them */
#include <tspdrvOutputDataHandler.c>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <VibeOSKernelLinuxHRTime.c>
#else
#include <VibeOSKernelLinuxTime.c>
#endif

/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
#if HAVE_UNLOCKED_IOCTL
static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif
static struct file_operations fops = 
{
    .owner =            THIS_MODULE,
    .read =             read,
    .write =            write,
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl =   unlocked_ioctl,
#else
    .ioctl =            ioctl,
#endif
    .open =             open,
    .release =          release,
    .llseek =           default_llseek    /* using default implementation as declared in linux/fs.h */
};

#ifndef IMPLEMENT_AS_CHAR_DRIVER
static struct miscdevice miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     MODULE_NAME,
	.fops =     &fops
};
#endif

#ifdef CONFIG_PM
static int suspend(struct device *dev);
static int resume(struct device *dev);
static const struct dev_pm_ops ts5000_vib_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(suspend, resume)
};
#else
static int suspend(struct platform_device *pdev, pm_message_t state);
static int resume(struct platform_device *pdev);
#endif
static struct platform_driver platdrv = 
{
#ifndef CONFIG_PM
    .suspend =  suspend,	
    .resume =   resume,	
#endif
    .driver = 
    {		
        .name = MODULE_NAME,	
#ifdef CONFIG_PM
        .pm = &ts5000_vib_pm_ops,
#endif
    },	
};

static void platform_release(struct device *dev);
static struct platform_device platdev = 
{	
	.name =     MODULE_NAME,	
	.id =       -1,                     /* means that there is only one device */
	.dev = 
    {
		.platform_data = NULL, 		
		.release = platform_release,    /* a warning is thrown during rmmod if this is absent */
	},
};

static int open(struct inode *inode, struct file *file) 
{
    DbgOut("tspdrv: open.\n");

    if (!try_module_get(THIS_MODULE)) return -ENODEV;

    return 0; 
}

static int release(struct inode *inode, struct file *file) 
{
    DbgOut("tspdrv: release.\n");

    /* 
    ** Reset force and stop timer when the driver is closed, to make sure
    ** no dangling semaphore remains in the system, especially when the
    ** driver is run outside of immvibed for testing purposes.
    */
    VibeOSKernelLinuxStopTimer();

    /* 
    ** Clear the variable used to store the magic number to prevent 
    ** unauthorized caller to write data. TouchSense service is the only 
    ** valid caller.
    */
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

    return 0; 
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

    /* End of buffer, exit */
    if (0 == nBufSize) return 0;

    if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) 
    {
        /* Failed to copy all the data, exit */
        DbgOut("tspdrv: copy_to_user failed.\n");
        return 0;
    }

    /* Update file position and return copied buffer size */
    *ppos += nBufSize;
    return nBufSize;
}

static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    *ppos = 0;  /* file position not used, always set to 0 */

    /* 
    ** Prevent unauthorized caller to write data. 
    ** TouchSense service is the only valid caller.
    */
    if (file->private_data != (void*)TSPDRV_MAGIC_NUMBER) 
    {
        DbgOut("tspdrv: unauthorized write.\n");
        return 0;
    }

    /* Copy immediately the input buffer */
    if (0 != copy_from_user(g_cWriteBuffer, buf, count))
    {
        /* Failed to copy all the data, exit */
        DbgOut("tspdrv: copy_from_user failed.\n");
        return 0;
    }

    /* Extract force output samples and save them in an internal buffer */
    if (!SaveOutputData(g_cWriteBuffer, count))
    {
        DbgOut("tspdrv: SaveOutputData failed.\n");
        return 0;
    }

    /* Start the timer after receiving new output force */
    g_bIsPlaying = true;

    VibeOSKernelLinuxStartTimer();

    return count;
}

#if HAVE_UNLOCKED_IOCTL
static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
    DbgOut("cmd=%d arg=%ld\n",cmd,arg);
    switch (cmd)
    {
        case TSPDRV_SET_MAGIC_NUMBER:
            file->private_data = (void*)TSPDRV_MAGIC_NUMBER;
            break;

        case TSPDRV_ENABLE_AMP:
            ImmVibeSPI_ForceOut_AmpEnable(arg);
            DbgRecorderReset((arg));
            DbgRecord((arg,";------- TSPDRV_ENABLE_AMP ---------\n"));
            break;

        case TSPDRV_DISABLE_AMP:
            ImmVibeSPI_ForceOut_AmpDisable(arg);
            break;

        case TSPDRV_GET_NUM_ACTUATORS:
            return NUM_ACTUATORS;

        case TSPDRV_SET_DEVICE_PARAMETER:
            {
                device_parameter deviceParam;

                if (0 != copy_from_user((void *)&deviceParam, (const void __user *)arg, sizeof(deviceParam)))
                {
                    /* Error copying the data */
                    DbgOut("tspdrv: copy_from_user failed to copy kernel parameter data.\n");
                    return -1;
                }

                switch (deviceParam.nDeviceParamID)
                {
                    case VIBE_KP_CFG_UPDATE_RATE_MS:
                        /* Update the timer period */
                        g_nTimerPeriodMs = deviceParam.nDeviceParamValue;



#ifdef CONFIG_HIGH_RES_TIMERS
                        /* For devices using high resolution timer we need to update the ktime period value */
                        g_ktTimerPeriod = ktime_set(0, g_nTimerPeriodMs * 1000000);
#endif
                        break;

                    case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    case VIBE_KP_CFG_FREQUENCY_PARAM6:
                        if (0 > ImmVibeSPI_ForceOut_SetFrequency(deviceParam.nDeviceIndex, deviceParam.nDeviceParamID, deviceParam.nDeviceParamValue))
                        {
                            DbgOut("tspdrv: cannot set device frequency parameter.\n");
                            return -1;
                        }
                        break;
                }
            }
        }
    return 0;
}

#ifdef CONFIG_PM
static int suspend(struct device *dev)
#else
static int suspend(struct platform_device *pdev, pm_message_t state) 
#endif
{
    if (g_bIsPlaying)
    {
        DbgOut("tspdrv: can't suspend, still playing effects.\n");
        return -EBUSY;
    }
    else
    {
        DbgOut("tspdrv: suspend.\n");
        return 0;
    }
}

#ifdef CONFIG_PM
static int resume(struct device *dev) 
#else
static int resume(struct platform_device *pdev) 
#endif
{	
    DbgOut("tspdrv: resume.\n");

	return 0;   /* can resume */
}

static void platform_release(struct device *dev) 
{	
    DbgOut("tspdrv: platform_release.\n");
}

static int ts5000_pm(bool enable)
{
    int nRet;
    struct regulator *vreg_lvs4_1p8;

    vreg_lvs4_1p8 = regulator_get(NULL, "8921_lvs4");
    if(IS_ERR(vreg_lvs4_1p8)) {
        nRet = PTR_ERR(vreg_lvs4_1p8);
        DbgOut(KERN_ERR "regulator get of %s failed (%d)\n", "8921_lvs4", nRet);
        return -EIO;
    }
    if(enable)
        nRet = regulator_enable(vreg_lvs4_1p8);
    else
        nRet = regulator_disable(vreg_lvs4_1p8);
    msleep(50);
    if(nRet<0) {
        DbgOut(KERN_ERR "fail to %s regulator 8921_lvs4 (%d)\n",enable ? "enable" : "disable", nRet);
        return -EIO;
    }
    regulator_put(vreg_lvs4_1p8);

    return nRet;
}

static int __init ts5000_init(void)
{
    int nRet, i;   /* initialized below */

    DbgOut("tspdrv: init_module.\n");

    ts5000_pm(true);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    g_nMajor = register_chrdev(0, MODULE_NAME, &fops);
    if (g_nMajor < 0) 
    {
        DbgOut("tspdrv: can't get major number.\n");
        return g_nMajor;
    }
#else
    nRet = misc_register(&miscdev);
	if (nRet) 
    {
        DbgOut("tspdrv: misc_register failed.\n");
		return nRet;
	}
#endif

	nRet = platform_device_register(&platdev);
	if (nRet) 
    {
        DbgOut("tspdrv: platform_device_register failed.\n");
    }

	nRet = platform_driver_register(&platdrv);
	if (nRet) 
    {
        DbgOut("tspdrv: platform_driver_register failed.\n");
    }

    DbgRecorderInit(());

    ImmVibeSPI_ForceOut_Initialize();
    VibeOSKernelLinuxInitTimer();
    ResetOutputData();

    /* Get and concatenate device name and initialize data buffer */
    g_cchDeviceName = 0;
    for (i=0; i<NUM_ACTUATORS; i++)
    {
        char *szName = g_szDeviceName + g_cchDeviceName;
        ImmVibeSPI_Device_GetName(i, szName, VIBE_MAX_DEVICE_NAME_LENGTH);

        /* Append version information and get buffer length */
        strcat(szName, VERSION_STR);
        g_cchDeviceName += strlen(szName);

    }

    return 0;
}

static void __exit ts5000_exit(void)
{
    DbgOut("tspdrv: cleanup_module.\n");

    DbgRecorderTerminate(());

    VibeOSKernelLinuxTerminateTimer();
    ImmVibeSPI_ForceOut_Terminate();

	platform_driver_unregister(&platdrv);
	platform_device_unregister(&platdev);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    unregister_chrdev(g_nMajor, MODULE_NAME);
#else
    misc_deregister(&miscdev);
#endif
}
module_init(ts5000_init);
module_exit(ts5000_exit);

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");

