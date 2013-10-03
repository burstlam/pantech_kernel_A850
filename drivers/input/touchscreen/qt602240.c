/* file_name: qt602240.c
 *
 * description: Quantum TSP driver.
 *
 * Copyright (C) 2008-2010 Atmel & Pantech Co. Ltd.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/uaccess.h>

#include "qt602240.h"
#include "pantech_touch_cmd.h"
/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */

static int DebugON=0;
#define dbg(fmt, args...) if(DebugON)printk("[QT602240]" fmt, ##args)

#define dbg_func_in()		dbg("[+] %s\n", __func__)
#define dbg_func_out()		dbg("[-] %s\n", __func__)
#define dbg_line()		dbg("line : %d | func : %s\n", __LINE__, __func__)
/* -------------------------------------------------------------------- */

//#define IRQ_TOUCH_INT		TEGRA_GPIO_TO_IRQ(GPIO_TOUCH_CHG)
/* -------------------------------------------------------------------- */

#define IS_QT602240_MXT224E(x)	1//((x != MXT224_FID) ? 1:0)
#define IS_QT602240_MXT768E(x)	0//((x != MXT768E_FID) ? 1:0)

#ifdef QT_FIRMUP_ENABLE
/* -------------------------------------------------------------------- */
/* qt602240 chipset firmware data to update */
/* -------------------------------------------------------------------- */
unsigned char QT602240_firmware[] = {
#include "qt602240_2_0AA.h"
};

unsigned char QT602240_E_firmware[] = {
#include "mXT224e__APP_v1_0_AA.h"
};

uint8_t i2c_addresses[] =
{
	I2C_APPL_ADDR_0,
	I2C_APPL_ADDR_1,
	I2C_BOOT_ADDR_0,
	I2C_BOOT_ADDR_1,
};

uint8_t	QT_i2c_address;

void	QT_reprogram(void);
uint8_t	QT_Boot(bool withReset);
/* -------------------------------------------------------------------- */
#endif // QT_FIRMUP_ENABLE


#define IRQ_TOUCH_INT       gpio_to_irq(GPIO_TOUCH_CHG)

/* -------------------------------------------------------------------- */
/* function proto type & variable for driver							*/
/* -------------------------------------------------------------------- */
static int __devinit qt602240_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit qt602240_remove(struct i2c_client *client);
#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
static int qt602240_late_resume(struct early_suspend *h);
static int qt602240_early_suspend(struct early_suspend *h);
#endif //CONFIG_PM && CONFIG_HAS_EARLYSUSPEND

static struct class *touch_atmel_class;
struct device *ts_dev;

static const struct i2c_device_id qt602240_id[] = {
	{ "qt602240-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qt602240_id);

static struct i2c_driver qt602240_driver = {
	.driver = {
		.name	= "qt602240-i2c",
		.owner	= THIS_MODULE,
	},
	.probe		= qt602240_probe,
	.remove		= __devexit_p(qt602240_remove),
	.id_table	= qt602240_id,
//#if defined(CONFIG_PM) 
//	.suspend    	= qt602240_suspend,
//	.resume		= qt602240_resume,
//#endif //CONFIG_PM
};

struct workqueue_struct *qt602240_wq;

struct qt602240_data_t
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend es;
	struct mutex    lock;
};
struct qt602240_data_t *qt602240_data = NULL;
/* -------------------------------------------------------------------- */


/* -------------------------------------------------------------------- */
/* function proto type & variable for attribute				*/
/* -------------------------------------------------------------------- */
/* for attribute */
static ssize_t i2c_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t i2c_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size);
static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t gpio_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size);
static ssize_t setup_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t setup_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size);
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static DEVICE_ATTR(i2c, S_IRUGO | S_IWUSR, i2c_show, i2c_store);
static DEVICE_ATTR(setup, S_IRUGO | S_IWUSR, setup_show, setup_store);
/* -------------------------------------------------------------------- */


typedef struct
{
	info_id_t info_id;          /*! Info ID struct. */
	object_t *objects;          /*! Pointer to an array of objects. */
	uint16_t CRC;               /*! CRC field, low bytes. */
	uint8_t CRC_hi;             /*! CRC field, high byte. */
} info_block_t;


typedef struct
{
	uint8_t object_type;     /*!< Object type. */
	uint8_t instance;        /*!< Instance number. */
} report_id_map_t;

#ifdef QT_MULTITOUCH_ENABLE
typedef struct
{
	uint8_t id;                     /*!< id */
	int8_t status;          /*!< dn=1, up=0, none=-1 */
	int16_t x;                      /*!< X */
	int16_t y;                      /*!< Y */

	int8_t mode;
	int16_t realx;                     
	int16_t realy;
	int area;
} report_finger_info_t;

typedef struct
{
	uint16_t code;                      /*!< key code */
	uint8_t status;                      /*!< key status (press/release) */
	bool update;     
} report_key_info_t;

static report_finger_info_t fingerInfo[MAX_NUM_FINGER];
static report_key_info_t keyInfo[MAX_NUM_FINGER];
#endif //QT_MULTITOUCH_ENABLE

static info_block_t *info_block;
static report_id_map_t *report_id_map;
volatile uint8_t read_pending;
static int max_report_id = 0;
uint8_t max_message_length;
uint16_t message_processor_address;

/*! Command processor address. */
static uint16_t command_processor_address;

/*! Flag indicating if driver setup is OK. */
static enum driver_setup_t driver_setup = DRIVER_SETUP_INCOMPLETE;

/*! Message buffer holding the latest message received. */
uint8_t *quantum_msg;

/*! \brief The current address pointer. */
static U16 address_pointer;

static uint8_t tsp_version;
static uint8_t tsp_family_id;

/* flag to indicate if last calibration has been confirmed good */
static uint8_t cal_check_flag = 0u;

#ifdef CHIP_NOINIT
static bool Chip_NoInit = false;
#endif

/* Calibration checking routine calling code - in interrupt handler */
#ifdef RECHECK_AFTER_CALIBRATION
static uint8_t calibration_message = 0;
#endif

static uint8_t timer_tick = 0;
static struct timer_list timer;
static bool timer_enabled = false;
static uint8_t facesup_flag = 0;

static bool active_event = true;

static struct timer_list timer_chk_fhe;
static bool noise_setting_flag = false;
static bool timer_tick_fhe_elapse = false;
static bool fherr_during_noise_timer = false;

static bool is_cal_success = false;

static bool touch_diagnostic_ret = true;

#ifndef min
#define min(a,b) ((a)>(b)?(b):(a))
#endif
#ifndef max
#define max(a,b) ((b)>(a)?(b):(a))
#endif

typedef enum
{
	TSC_EVENT_NONE,
	TSC_EVENT_WINDOW,
	TSC_EVENT_MENU,
	TSC_EVENT_HOME,
	TSC_EVENT_BACK,  
	TSC_EVENT_FACEPRESS, 
} tsc_key_mode_type;

typedef enum
{
	TSC_CLEAR_ALL,
	TSC_CLEAR_EVENT,
} tsc_clear_type;

#ifdef TOUCH_IO
typedef enum 
{
	DIAGNOSTIC_REFERENCE_DEBUG= 501,
	DIAGNOSTIC_DELTA_DEBUG,
	DIAGNOSTIC_MIN_DEBUG,
	DIAGNOSTIC_MAX_DEBUG,

	POWER_CONFIG_IDLEACQINT=700,
	POWER_CONFIG_ACTVACQINT,
	POWER_CONFIG_ACTV2IDLETO,        

	ACQUISITION_CONFIG_CHRGTIME=800,
	ACQUISITION_CONFIG_ATCHDRIFT,
	ACQUISITION_CONFIG_TCHDRIFT,
	ACQUISITION_CONFIG_DRIFTST,
	ACQUISITION_CONFIG_TCHAUTOCAL,
	ACQUISITION_CONFIG_SYNC,
	ACQUISITION_CONFIG_ATCHCALST,
	ACQUISITION_CONFIG_ATCHCALSTHR,
	ACQUISITION_CONFIG_ATCHFRCCALTHR,
	ACQUISITION_CONFIG_ATCHFRCCALRATION,
	
	TOUCHSCREEN_CONFIG_CTRL=900,
	TOUCHSCREEN_CONFIG_XORIGIN,
	TOUCHSCREEN_CONFIG_YORIGIN,
	TOUCHSCREEN_CONFIG_XSIZE,
	TOUCHSCREEN_CONFIG_YSIZE,
	TOUCHSCREEN_CONFIG_AKSCFG,
	TOUCHSCREEN_CONFIG_BLEN,
	TOUCHSCREEN_CONFIG_TCHTHR,
	TOUCHSCREEN_CONFIG_TCHDI,
	TOUCHSCREEN_CONFIG_ORIENT,
	TOUCHSCREEN_CONFIG_MRGTIMEOUT,
	TOUCHSCREEN_CONFIG_MOVHYSTI,
	TOUCHSCREEN_CONFIG_MOVHYSTN,
	TOUCHSCREEN_CONFIG_MOVFILTER,
	TOUCHSCREEN_CONFIG_NUMTOUCH,
	TOUCHSCREEN_CONFIG_MRGHYST,
	TOUCHSCREEN_CONFIG_MRGTHR,
	TOUCHSCREEN_CONFIG_AMPHYST,
	TOUCHSCREEN_CONFIG_XRANGE,
	TOUCHSCREEN_CONFIG_YRANGE,
	TOUCHSCREEN_CONFIG_XLOCLIP,
	TOUCHSCREEN_CONFIG_XHICLIP,
	TOUCHSCREEN_CONFIG_YLOCLIP,
	TOUCHSCREEN_CONFIG_YHICLIP,
	TOUCHSCREEN_CONFIG_XEDGECTRL,
	TOUCHSCREEN_CONFIG_XEDGEDIST,
	TOUCHSCREEN_CONFIG_YEDGECTRL,
	TOUCHSCREEN_CONFIG_YEDGEDIST,
	TOUCHSCREEN_CONFIG_JUMPLIMIT,

	TOUCHSCREEN_CONFIG_TCHHYST,
	TOUCHSCREEN_CONFIG_XPITCH,
	TOUCHSCREEN_CONFIG_YPITCH,
	TOUCHSCREEN_CONFIG_NEXTTCHDI,

	KEYARRAY_CONFIG_CTRL=1500,
	KEYARRAY_CONFIG_XORIGIN,
	KEYARRAY_CONFIG_YORIGIN,
	KEYARRAY_CONFIG_XSIZE,
	KEYARRAY_CONFIG_YSIZE,
	KEYARRAY_CONFIG_AKSCFG,
	KEYARRAY_CONFIG_BLEN,
	KEYARRAY_CONFIG_TCHTHR,
	KEYARRAY_CONFIG_TCHDI,

	COMC_CONFIG_CTRL=1800,
	COMC_CONFIG_COMMAND,

	GPIOPWM_CONFIG_CTRL=1900,
	GPIOPWM_CONFIG_REPORTMASK,
	GPIOPWM_CONFIG_DIR,
	GPIOPWM_CONFIG_INTPULLUP,
	GPIOPWM_CONFIG_OUT,
	GPIOPWM_CONFIG_WAKE,
	GPIOPWM_CONFIG_PWM,
	GPIOPWM_CONFIG_PERIOD,
	GPIOPWM_CONFIG_DUTY0,
	GPIOPWM_CONFIG_DUTY1,
	GPIOPWM_CONFIG_DUTY2,
	GPIOPWM_CONFIG_DUTY3,
	GPIOPWM_CONFIG_TRIGGER0,
	GPIOPWM_CONFIG_TRIGGER1,
	GPIOPWM_CONFIG_TRIGGER2,
	GPIOPWM_CONFIG_TRIGGER3,

	PROXIMITY_CONFIG_CTRL=2300,
	PROXIMITY_CONFIG_XORIGIN,
	PROXIMITY_CONFIG_YORIGIN,
	PROXIMITY_CONFIG_XSIZE,
	PROXIMITY_CONFIG_YSIZE,
	PROXIMITY_CONFIG_BLEN,
	PROXIMITY_CONFIG_FXDDTHR,
	PROXIMITY_CONFIG_FXDDI,
	PROXIMITY_CONFIG_AVERAGE,
	PROXIMITY_CONFIG_MVNULLRATE,
	PROXIMITY_CONFIG_MVDTHR,

	ONETOUCH_GESTURE_CONFIG_CTRL=2400,
	ONETOUCH_GESTURE_CONFIG_NUMGEST,
	ONETOUCH_GESTURE_CONFIG_GESTEN,
	ONETOUCH_GESTURE_CONFIG_PRESSPROC,
	ONETOUCH_GESTURE_CONFIG_TAPTO,
	ONETOUCH_GESTURE_CONFIG_FLICKTO,
	ONETOUCH_GESTURE_CONFIG_DRAGTO,
	ONETOUCH_GESTURE_CONFIG_SPRESSTO,
	ONETOUCH_GESTURE_CONFIG_LPRESSTO,
	ONETOUCH_GESTURE_CONFIG_REPPRESSTO,
	ONETOUCH_GESTURE_CONFIG_FLICKTHR,
	ONETOUCH_GESTURE_CONFIG_DRAGTHR,
	ONETOUCH_GESTURE_CONFIG_TAPTHR,
	ONETOUCH_GESTURE_CONFIG_THROWTHR,
	ONETOUCH_GESTURE_CONFIG_PROCESS,

	SELFTEST_CONFIG_CTRL=2500,
	SELFTEST_CONFIG_CMD,

	TWOTOUCH_GESTURE_CONFIG_CTRL=2700,
	TWOTOUCH_GESTURE_CONFIG_NUMGEST,
	TWOTOUCH_GESTURE_CONFIG_GESTEN,
	TWOTOUCH_GESTURE_CONFIG_ROTATETHR,
	TWOTOUCH_GESTURE_CONFIG_ZOOMTHR,

	GRIPFACESUPPRESSION_CONFIG_CTRL=4000,
	GRIPFACESUPPRESSION_CONFIG_XLOGRIP,
	GRIPFACESUPPRESSION_CONFIG_XHIGRIP,
	GRIPFACESUPPRESSION_CONFIG_YLOGRIP,
	GRIPFACESUPPRESSION_CONFIG_YHIGRIP,
	GRIPFACESUPPRESSION_CONFIG_MAXTCHS,
	GRIPFACESUPPRESSION_CONFIG_RESERVED,
	GRIPFACESUPPRESSION_CONFIG_SZTHR1,
	GRIPFACESUPPRESSION_CONFIG_SZTHR2,
	GRIPFACESUPPRESSION_CONFIG_SHPTHR1,
	GRIPFACESUPPRESSION_CONFIG_SHPTHR2,
	GRIPFACESUPPRESSION_CONFIG_SUPEXTTO,

	TOUCHSUPPRESSION_CTRL = 4200,
	TOUCHSUPPRESSION_APPRTHR,
	TOUCHSUPPRESSION_MAXAPPRAREA,
	TOUCHSUPPRESSION_MAXTCHAREA,
	TOUCHSUPPRESSION_SUPSTRENGTH,
	TOUCHSUPPRESSION_SUPEXTTO,
	TOUCHSUPPRESSION_MAXNUMTCHS,
	TOUCHSUPPRESSION_SHAPESTRENGTH,

	CTE_CONFIG_CTRL=4600,
	CTE_CONFIG_MODE,
	CTE_CONFIG_IDLEGCAFDEPTH,
	CTE_CONFIG_ACTVGCAFDEPTH,
	CTE_CONFIG_ADCSPERSYNC,
	CTE_CONFIG_PULSESPERADC,
	CTE_CONFIG_XSLEW,
	CTE_CONFIG_SYNCDELAY,

	STYLUS_CTRL = 4700,
	STYLUS_CONTMIN,
	STYLUS_CONTMAX,
	STYLUS_STABILITY,
	STYLUS_MAXTCHAREA,
	STYLUS_AMPLTHR,
	STYLUS_STYSHAPE,
	STYLUS_HOVERSUP,
	STYLUS_CONFTHR,
	STYLUS_SYNCSPERX,

	NOISESUPPRESSION_CTRL = 4800,
	NOISESUPPRESSION_CFG,
	NOISESUPPRESSION_CALCFG,
	NOISESUPPRESSION_BASEFREQ,
	NOISESUPPRESSION_MFFREQ0,
	NOISESUPPRESSION_MFFREQ1,
	NOISESUPPRESSION_GCACTVINVLDADCS,
	NOISESUPPRESSION_GCIDLEINVLDADCS,
	NOISESUPPRESSION_GCMAXADCSPERX,
	NOISESUPPRESSION_GCLIMITMIN,
	NOISESUPPRESSION_GCLIMITMAX,
	NOISESUPPRESSION_GCCOUNTMINTGT,
	NOISESUPPRESSION_MFINVLDDIFFTHR,
	NOISESUPPRESSION_MFINCADCSPXTHR,
	NOISESUPPRESSION_MFERRORTHR,
	NOISESUPPRESSION_SELFREQMAX,
	NOISESUPPRESSION_BLEN,
	NOISESUPPRESSION_TCHTHR,
	NOISESUPPRESSION_TCHDI,
	NOISESUPPRESSION_MOVHYSTI,
	NOISESUPPRESSION_MOVHYSTN,
	NOISESUPPRESSION_MOVFILTER,
	NOISESUPPRESSION_NUMTOUCH,
	NOISESUPPRESSION_MRGHYST,
	NOISESUPPRESSION_MRGTHR,
	NOISESUPPRESSION_XLOCLIP,
	NOISESUPPRESSION_XHICLIP,
	NOISESUPPRESSION_YLOCLIP,
	NOISESUPPRESSION_YHICLIP,
	NOISESUPPRESSION_XEDGECTRL,
	NOISESUPPRESSION_XEDGEDIST,
	NOISESUPPRESSION_YEDGECTRL,
	NOISESUPPRESSION_YEDGEDIST,
	NOISESUPPRESSION_JUMPLIMIT,
	NOISESUPPRESSION_TCHHYST,
	NOISESUPPRESSION_NEXTTCHDI,

	GENDATASOURCE_DSTYPE=5300,
	GENDATASOURCE_XORIGIN,
	GENDATASOURCE_XSIZE,
	GENDATASOURCE_YORIGIN,
	GENDATASOURCE_YSIZE,

	APPLY_TOUCH_CONFIG=9000,
}CONFIG_CMD;
#endif //TOUCH_IO

#ifdef CHARGER_MODE
typedef enum
{
	BATTERY_PLUGGED_NONE		= 0,
	BATTERY_PLUGGED_AC		= 1,
	BATTERY_PLUGGED_USB		= 2
}type_charger_mode;

unsigned long pre_charger_mode;
#endif //CHARGER_MODE

unsigned int stylus_mode = false;

#ifdef CHECK_FHE
typedef enum
{
	FHE_CLEAR = 0,
	FHE_SET = 1,
} FHE_MODE;
#endif

static int nCalCount=0;

static int reference_data[QT602240_PAGE_NUM*QT602240_PAGE_SIZE/2] = { 0 };
static int diagnostic_min =0;
static int diagnostic_max =0;
static bool charger_mode_flag = true;

typedef enum
{
	AT_STOP = 0,
	AT_START = 1,
	AT_ENABLE_CNT = 2,
	AT_DISABLE_CNT = 3,
} AUTOTOUCH_MODE;

#define MAX_AUTO_CNT 5	// sec

static int auto_touch_flag = AT_STOP;
static int auto_touch_cnt = 0;

/*------------------------------ functions prototype -----------------------------------*/
uint8_t init_touch_driver(uint8_t I2C_address);
void message_handler(U8 *msg, U8 length);
uint8_t reset_chip(void);
uint8_t calibrate_chip(void);
uint8_t diagnostic_chip(uint8_t mode);
uint8_t backup_config(void);
uint8_t get_variant_id(uint8_t *variant);
uint8_t get_family_id(void);
uint8_t get_build_number(uint8_t *build);
uint8_t get_version(uint8_t *version);
uint8_t write_power_config(gen_powerconfig_t7_config_t power_config);
uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t acq_config);
uint8_t write_multitouchscreen_config(uint8_t screen_number, touch_multitouchscreen_t9_config_t cfg);
uint8_t write_keyarray_config(uint8_t key_array_number, touch_keyarray_t15_config_t cfg);
uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg);
uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg);
uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg);
uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg);
uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg);
uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg);
uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg);
uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg);
uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg);
uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg);
uint8_t get_object_size(uint8_t object_type);
uint8_t type_to_report_id(uint8_t object_type, uint8_t instance);
uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);
uint8_t read_id_block(info_id_t *id);
uint16_t get_object_address(uint8_t object_type, uint8_t instance);
uint32_t get_stored_infoblock_crc(void);
uint8_t calculate_infoblock_crc(uint32_t *crc_pointer);
uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2);
void write_message_to_usart(uint8_t msg[], uint8_t length);
static int init_hw_setting(void);
void TSP_Restart(void);
int8_t check_chip_calibration(void);

#if defined(__MXT224E_CONFIG__)
uint8_t write_gripsuppression_T40_config(proci_gripsuppression_t40_config_t cfg);
uint8_t write_palmsuppression_T41_config(proci_palmsuppression_t41_config_t cfg);
uint8_t write_touchsuppression_t42_config(proci_touchsuppression_t42_config_t cfg);
uint8_t write_digitizer_t43_config(spt_digitizer_t43_config_t cfg);

uint8_t write_CTE_T46_config(spt_cteconfig_t46_config_t cfg);
uint8_t  write_stylus_t47_config(proci_stylus_t47_config_t cfg);
uint8_t  write_noisesuppression_t48_config(procg_noisesuppression_t48_config_t cfg);
#endif /*__MXT224E_CONFIG__ */

uint8_t  write_proximitykey_t52_config(proximitykey_t52_config_t cfg);
uint8_t  write_gendatasource_t53_config(gendatasource_t53_config_t cfg);

U8 read_changeline(void);
U8 init_I2C(U8 I2C_address_arg);
U8 read_mem(U16 start, U8 size, U8 *mem);
U8 read_U16(U16 start, U16 *mem);
U8 write_mem(U16 start, U8 size, U8 *mem);

void read_all_register(void);

void  clear_event(uint8_t clear);


#ifdef SKY_PROCESS_CMD_KEY
static long ts_fops_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static int ts_fops_open(struct inode *inode, struct file *filp);
#endif

#ifdef TOUCH_IO
/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static long ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static void apply_touch_config(void);
#ifdef CHARGER_MODE
static uint8_t qt_charger_mode_config(unsigned long mode);
#endif // CHARGER_MODE
#endif // TOUCH_IO

void  qt602240_front_test_init(void);
void off_hw_setting(void);

static void pantech_stylus_mode_config(int mode);
static int* pantech_diagnostic_debug(int command) ;

/*------------------------------ for tunning ATmel - start ----------------------------*/
struct class *touch_class;
EXPORT_SYMBOL(touch_class);

struct device *switch_test;
EXPORT_SYMBOL(switch_test);


/******************************************************************************
 * 
 *       QT602240 Object table init
 *                                                             
 * *****************************************************************************/
//General Object
gen_powerconfig_t7_config_t power_config = {0};                 //Power config settings.
gen_acquisitionconfig_t8_config_t acquisition_config = {0};     // Acquisition config. 

//Touch Object
touch_multitouchscreen_t9_config_t touchscreen_config = {0};    //Multitouch screen config.
touch_keyarray_t15_config_t keyarray_config = {0};              //Key array config.
touch_proximity_t23_config_t proximity_config = {0};        //Proximity config. 

//Signal Processing Objects
proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};    //Grip / face suppression config.
procg_noisesuppression_t22_config_t noise_suppression_config = {0};         //Noise suppression config.
proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};  //One-touch gesture config. 
proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};  //Two-touch gesture config.

//Support Objects
spt_comcconfig_t18_config_t   comc_config = {0};            //Communication config settings.
spt_gpiopwm_t19_config_t  gpiopwm_config = {0};             //GPIO/PWM config
spt_selftest_t25_config_t selftest_config = {0};            //Selftest config.
spt_cteconfig_t28_config_t cte_config = {0};                //Capacitive touch engine config.

#if defined(__MXT224E_CONFIG__)
//MXT224E Objects
proci_gripsuppression_t40_config_t              gripsuppression_t40_config = {0};       
proci_touchsuppression_t42_config_t             touchsuppression_t42_config = {0};
spt_cteconfig_t46_config_t                      cte_t46_config = {0};
proci_stylus_t47_config_t                       stylus_t47_config = {0};
procg_noisesuppression_t48_config_t             noisesuppression_t48_config = {0};
#endif

spt_digitizer_t43_config_t			spt_digitizer_t43_config = {0};
proximitykey_t52_config_t			proximitykey_t52_config = {0};

gendatasource_t53_config_t			gendatasource_t53_config = {0};

/*------------------------------ for tunning ATmel - end ----------------------------*/

#ifdef PANTECH_GPIO_KEY

#define GPIO_VOL_DOWN	40
#define GPIO_VOL_UP		71

static irqreturn_t pantech_gpio_vol_up_key(int irq, void *handle)
{
	int rt =0;

	rt = gpio_get_value(GPIO_VOL_UP);

//	printk("pantech_gpio_vol_up_key value: %d\n", rt);

	if(rt)
		input_report_key(qt602240_data->input_dev, KEY_VOLUMEUP, 0);
	else
		input_report_key(qt602240_data->input_dev, KEY_VOLUMEUP, 1);

	input_sync(qt602240_data->input_dev);
	
	return IRQ_HANDLED;
}

static irqreturn_t pantech_gpio_vol_down_key(int irq, void *handle)
{
	int rt =0;

	rt = gpio_get_value(GPIO_VOL_DOWN);

//	printk("pantech_gpio_vol_down_key value: %d\n", rt);

	if(rt)
		input_report_key(qt602240_data->input_dev, KEY_VOLUMEDOWN, 0);
	else
		input_report_key(qt602240_data->input_dev, KEY_VOLUMEDOWN, 1);

	input_sync(qt602240_data->input_dev);
	
	return IRQ_HANDLED;
}

#endif

#define __QT_CONFIG__

#ifdef SKY_PROCESS_CMD_KEY
static struct file_operations ts_fops = {
	.owner = THIS_MODULE,
	.open = ts_fops_open,
	//	.release = ts_fops_close,
	.unlocked_ioctl = ts_fops_ioctl,
};

static struct miscdevice touch_event = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fops",
	.fops = &ts_fops,
};

static int ts_fops_open(struct inode *inode, struct file *filp)
{
	filp->private_data = qt602240_data;
	return 0;
}

static long ts_fops_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	dbg("[QT602240]ts_fops_ioctl(%d, %d) \n",(int)cmd,(int)arg);
	
	switch (cmd) 
	{
		case TOUCH_IOCTL_READ_LASTKEY:
			break;
		case TOUCH_IOCTL_DO_KEY:
			dbg("TOUCH_IOCTL_DO_KEY  = %d\n",(int)argp);			
			if ( (int)argp == 0x20a )
				input_report_key(qt602240_data->input_dev, 0xe3, 1);
			else if ( (int)argp == 0x20b )
				input_report_key(qt602240_data->input_dev, 0xe4, 1);
			else
				input_report_key(qt602240_data->input_dev, (int)argp, 1);
		
			break;
		case TOUCH_IOCTL_RELEASE_KEY:		
			dbg("TOUCH_IOCTL_RELEASE_KEY  = %d\n",(int)argp);
			if ( (int)argp == 0x20a )
				input_report_key(qt602240_data->input_dev, 0xe3, 0);
			else if ( (int)argp == 0x20b )
				input_report_key(qt602240_data->input_dev, 0xe4, 0);
			else
				input_report_key(qt602240_data->input_dev, (int)argp, 0);
			break;		
		case TOUCH_IOCTL_DEBUG:
			dbg("Touch Screen Read Queue ~!!\n");	
			queue_work(qt602240_wq, &qt602240_data->work);

			dbg("[TSP] TOUCH_IOCTL_DEBUG auto_touch_flag %d\n", auto_touch_flag);

			if(auto_touch_flag == AT_STOP)
			{
				auto_touch_flag = AT_START;		// enable auto touch
				auto_touch_cnt = 0;
			}
			else if(auto_touch_flag == AT_ENABLE_CNT)
			{
				if(auto_touch_cnt > MAX_AUTO_CNT)
				{
					acquisition_config.atchcalst = 50;
					acquisition_config.atchcalsthr = 100; 
					noisesuppression_t48_config.cfg = T48_CFG;

					printk("[TSP] recover atchcalst=%d, atchcalsthr=%d\n", acquisition_config.atchcalst, acquisition_config.atchcalsthr );

					/* Write temporary acquisition config to chip. */
					if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
						printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			
					if (write_noisesuppression_t48_config(noisesuppression_t48_config) != CFG_WRITE_OK)
					{
						printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
					}	

					auto_touch_flag = AT_DISABLE_CNT;
					auto_touch_cnt = 0;
					calibrate_chip();
				}
				else
				{
					auto_touch_cnt++;
				}
			}
			
			break;
		case TOUCH_IOCTL_CLEAN:
			dbg("Touch Screen Previous Data Clean ~!!\n");
			clear_event(TSC_CLEAR_ALL);
			break;
		case TOUCH_IOCTL_RESTART:
			dbg("Touch Screen Calibration Restart ~!!\n");			
			calibrate_chip();
			break;
		case TOUCH_IOCTL_PRESS_TOUCH:
			input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, (int)(arg&0x0000FFFF));
			input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, (int)((arg >> 16) & 0x0000FFFF));
			input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, 255);
			input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(qt602240_data->input_dev);
			input_sync(qt602240_data->input_dev);
			break;
		case TOUCH_IOCTL_RELEASE_TOUCH:		
			input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, (int)(arg&0x0000FFFF));
			input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, (int)((arg >> 16) & 0x0000FFFF));
			input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(qt602240_data->input_dev);
			input_sync(qt602240_data->input_dev); 
			break;
		case TOUCH_IOCTL_CHARGER_MODE:

			if(pre_charger_mode == arg || charger_mode_flag == false)
				break;
	
			pre_charger_mode = arg;
			
			qt_charger_mode_config(arg);
			break;
		case POWER_OFF:
			pm_power_off();
			break;
		case TOUCH_IOCTL_DELETE_ACTAREA:
			touchscreen_config.yloclip = 0;		// Change Active area
			touchscreen_config.yhiclip = 0;
			if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
			{
				dbg("qt_Multitouchscreen_config Error!!!\n");
			}
			break;
		case TOUCH_IOCTL_RECOVERY_ACTAREA:
			touchscreen_config.yloclip = 15;	// Change Active area
			touchscreen_config.yhiclip = 15;
			if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
			{
				dbg("qt_Multitouchscreen_config Error!!!\n");
			}
			break;

		case TOUCH_IOCTL_STYLUS_MODE:

			printk("TOUCH_IOCTL_STYLUS_MODE Config %d\n", (int)argp);
			
			if((int)argp == true)
				stylus_mode = true;
			else
				stylus_mode = false;
			
			pantech_stylus_mode_config(pre_charger_mode);
			break;

		case TOUCH_CHARGE_MODE_CTL:

			printk("TOUCH_CHARGE_MODE_CTL Config %d\n", (int)argp);

			if(arg == true) // Enable Charger mode
			{
				charger_mode_flag = true;
			}
			else // Disable Chager mode
			{
				qt_charger_mode_config(BATTERY_PLUGGED_NONE);
				charger_mode_flag = false;
			}
			break;
			
		default:
			break;
	}

	return true;
}
#endif

#ifdef TOUCH_IO
void print_touch_info(void)
{
	info_id_t *id;
	id = (info_id_t *) kmalloc(sizeof(info_id_t), GFP_KERNEL | GFP_ATOMIC);
	read_id_block(id);
}
void print_touch_reg(void)
{
	dbg("touchscreen_config tchthr : %d\n",  touchscreen_config.tchthr);
	dbg("touchscreen_config tchdi : %d\n",  touchscreen_config.tchdi);
	dbg("touchscreen_config blen : %d\n",  touchscreen_config.blen);
	dbg("touchscreen_config orient : %d\n",  touchscreen_config.orient);
	dbg("touchscreen_config tchhyst : %d\n",  touchscreen_config.tchhyst);
	dbg("noisesuppression_t48_config blen : %d\n",  noisesuppression_t48_config.blen);
	dbg("noisesuppression_t48_config calcfg : %d\n",  noisesuppression_t48_config.calcfg);
	dbg("noisesuppression_t48_config cfg : %d\n",  noisesuppression_t48_config.cfg);
	dbg("noisesuppression_t48_config tchthr : %d\n",  noisesuppression_t48_config.tchthr);
	dbg("noisesuppression_t48_config tchdi : %d\n",  noisesuppression_t48_config.tchdi);
	dbg("noisesuppression_t48_config tchhyst : %d\n",  noisesuppression_t48_config.tchhyst);	
}
void reset_touch_info(void)
{
}


static int* pantech_diagnostic_debug(int command) 
{
	/*command 0x10: Delta, 0x11: Reference*/
	uint8_t data_buffer[QT602240_PAGE_SIZE] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0; /* dianostic command to get touch refs*/
	uint16_t diag_address;

	uint8_t page;
	int i=0, j=0, max_cnt =100;
	uint32_t value;

	dbg_func_in();
	disable_irq(qt602240_data->client->irq);

	touch_diagnostic_ret = true;

	if (driver_setup != DRIVER_SETUP_OK)
		return NULL;

	mutex_lock(&qt602240_data->lock);

	memset(reference_data, 0x00, sizeof(reference_data));

	diagnostic_min = 0;
	diagnostic_max = 0;

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	diagnostic_chip(command);
	msleep(20); 

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	memset( data_buffer , 0xFF, sizeof( data_buffer ) );
	diag_address = get_object_address(DEBUG_DIAGNOSTIC_T37,0);
	
	/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

	/* count up the channels/bits if we recived the data properly */
	//printk("[TSP] DIAG! \n");
	for (page = 0; page < QT602240_PAGE_NUM; page++) 
	{
		try_ctr=0;

		/* wait for diagnostic object to update */
		while(!((data_buffer[0] == command) && (data_buffer[1] == page)) && try_ctr++ < max_cnt)
		{
			read_mem(diag_address, 2,data_buffer);

			if(data_buffer[0] != command || data_buffer[1] != page)
				msleep(5); 
			
			//printk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
		}

		if(try_ctr >= max_cnt)
		{
			printk("[TSP] Error read diagnostic cmd: %d\n", command);
			mutex_unlock(&qt602240_data->lock);
			return NULL;
		}
		
		/* data is ready - read the detection flags */
		read_mem(diag_address, QT602240_PAGE_SIZE+2,data_buffer);
//		printk("[TSP] CMD: %x Page: %x \n", data_buffer[0],data_buffer[1]);
		
		if((data_buffer[0] == command) && (data_buffer[1] == page))
		{
			/* count the channels and print the flags to the log */
			for(i = 0; i < QT602240_PAGE_SIZE; i+=2) /* check X lines - data is in words so increment 2 at a time */
			{
				value =  (data_buffer[3+i]<<8) + data_buffer[2+i];
			
			        if (command == QT602240_REFERENCE_MODE)
				{
					reference_data[j] = value - 0x4000;

					if((reference_data[j] < QT602240_REFERENCE_MIN || reference_data[j] > QT602240_REFERENCE_MAX) && j < QT602240_MAX_CHANNEL_NUM)
						touch_diagnostic_ret = false;
			        }
				else
				{
					if(value > 10000)
						reference_data[j] = value - 65536;
					else
						reference_data[j] = value;

					diagnostic_min = min(diagnostic_min, reference_data[j] );
					diagnostic_max = max(diagnostic_max, reference_data[j] );					
				}
					
//				/* print the flags to the log - only really needed for debugging */
//				if(command == QT602240_REFERENCE_MODE && (reference_data[j]  < 4160 || reference_data[j]  > 10400))
//					printk("%d %d " , reference_data[j], j);
//				if((j % T9_XSIZE) == 0) 
//					printk("\n");

				j++;

				if(j > (QT602240_PAGE_NUM*QT602240_PAGE_SIZE/2))
					printk("[TSP]ERROR!! %d\n", j);

			}
		}
		else 
		{
			printk("[TSP]ERROR!!");
		}

		/* send page up command so we can detect when data updates next time,
		 * page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;
		write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);

		
	}

	enable_irq(qt602240_data->client->irq);
	dbg_func_out();

	mutex_unlock(&qt602240_data->lock);

	return (int *)reference_data;
}

static struct file_operations fops = 
{
	.owner 					=    THIS_MODULE,
	.unlocked_ioctl =    ioctl,
	.read 					=     read,
	.write 					=    write,
	.open 					=     open,
	.release 				=  release
};

static struct miscdevice touch_io = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "qt602240",
	.fops =     &fops
};

static int open(struct inode *inode, struct file *file) 
{
	return 0; 
}

static int release(struct inode *inode, struct file *file) 
{
	return 0; 
}
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int nBufSize=0;
	if((size_t)(*ppos) > 0) return 0;
	if(buf!=NULL)
	{
		nBufSize=strlen(buf);
		if(strncmp(buf, "queue", 5)==0)
		{
			queue_work(qt602240_wq, &qt602240_data->work);
		}
		if(strncmp(buf, "debug", 5)==0)
		{			
			DebugON=1;	 
			if(IS_QT602240_MXT224E(tsp_family_id)){
				dbg("This is MXT224 ECHO Version !!");	
			}else{
				dbg("This is MXT224 Another Version !!");
			}
		}
		if(strncmp(buf, "debugoff", 8)==0)
		{			
			DebugON=0;	    
		}
		if(strncmp(buf, "checkcal", 8)==0)
		{			
			check_chip_calibration();
		}
		if(strncmp(buf, "cal", 3)==0)
		{			
			calibrate_chip();
		}
		if(strncmp(buf, "resettouch", 10)==0)
		{			
			reset_touch_info();
		}
		if(strncmp(buf, "printtouch", 10)==0)
		{			
			print_touch_reg();
		}
		if(strncmp(buf, "infotouch", 9)==0)
		{			
			print_touch_info();
		}
		if(strncmp(buf, "save", 4)==0)
		{			
			backup_config();	    
		}
		if(strncmp(buf, "reference", 9)==0)
		{			
			pantech_diagnostic_debug(QT602240_REFERENCE_MODE);	    
		}
		if(strncmp(buf, "delta", 5)==0)
		{			
			pantech_diagnostic_debug(QT602240_DELTA_MODE);	    
		}
	}
	*ppos +=nBufSize;
	return nBufSize;
}
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return 0; 
}

#define TOUCH_CONFIG_READ 0xff

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

		switch (cmd)
		{
			case POWER_CONFIG_IDLEACQINT:
				if((int)arg == TOUCH_CONFIG_READ)
					return power_config.idleacqint;
				else
					power_config.idleacqint=arg;
				break;
			case POWER_CONFIG_ACTVACQINT:
				if((int)arg == TOUCH_CONFIG_READ)
					return power_config.actvacqint;
				else
					power_config.actvacqint=arg;
				break;
			case POWER_CONFIG_ACTV2IDLETO:        
				if((int)arg == TOUCH_CONFIG_READ)
					return power_config.actv2idleto;
				else
					power_config.actv2idleto=arg;
				break;

			case ACQUISITION_CONFIG_CHRGTIME:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.chrgtime;
				else				
					acquisition_config.chrgtime=arg;
				break;
			case ACQUISITION_CONFIG_ATCHDRIFT:
				break;
			case ACQUISITION_CONFIG_TCHDRIFT:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.tchdrift;
				else
					acquisition_config.tchdrift=arg;
				break;
			case ACQUISITION_CONFIG_DRIFTST:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.driftst;
				else
					acquisition_config.driftst = arg;
				break;
			case ACQUISITION_CONFIG_TCHAUTOCAL:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.tchautocal;
				else
					acquisition_config.tchautocal=arg;
				break;
			case ACQUISITION_CONFIG_SYNC:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.sync;
				else
					acquisition_config.sync =arg;
				break;
			case ACQUISITION_CONFIG_ATCHCALST:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.atchcalst;
				else
					acquisition_config.atchcalst=arg;
				break;
			case ACQUISITION_CONFIG_ATCHCALSTHR:   
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.atchcalsthr;
				else
					acquisition_config.atchcalsthr=arg;
				break;  
			case ACQUISITION_CONFIG_ATCHFRCCALTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.atchfrccalthr;
				else
					acquisition_config.atchfrccalthr=arg;
				break;  
			case ACQUISITION_CONFIG_ATCHFRCCALRATION:
				if((int)arg == TOUCH_CONFIG_READ)
					return acquisition_config.atchfrccalration;
				else
					acquisition_config.atchfrccalration=arg;
				break;  

			case TOUCHSCREEN_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.ctrl;
				else
					touchscreen_config.ctrl = arg;
				break;
			case TOUCHSCREEN_CONFIG_XORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xorigin;
				else
					touchscreen_config.xorigin =arg;
				break;
			case TOUCHSCREEN_CONFIG_YORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yorigin;
				else
					touchscreen_config.yorigin = arg;
				break;
			case TOUCHSCREEN_CONFIG_XSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xsize;
				else
					touchscreen_config.xsize = arg;
				break;
			case TOUCHSCREEN_CONFIG_YSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.ysize;
				else
					touchscreen_config.ysize = arg;
				break;
			case TOUCHSCREEN_CONFIG_AKSCFG:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.akscfg;
				else
					touchscreen_config.akscfg = arg;
				break;
			case TOUCHSCREEN_CONFIG_BLEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.blen;
				else
					touchscreen_config.blen = arg;
				break;
			case TOUCHSCREEN_CONFIG_TCHTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.tchthr ;
				else
					touchscreen_config.tchthr = arg;
				break;
			case TOUCHSCREEN_CONFIG_TCHDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.tchdi ;
				else
					touchscreen_config.tchdi = arg;
				break;
			case TOUCHSCREEN_CONFIG_ORIENT:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.orient;
				else
					touchscreen_config.orient = arg;
				break;
			case TOUCHSCREEN_CONFIG_MRGTIMEOUT:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.mrgtimeout;
				else
					touchscreen_config.mrgtimeout = arg;
				break;
			case TOUCHSCREEN_CONFIG_MOVHYSTI:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.movhysti;
				else
					touchscreen_config.movhysti = arg;
				break;
			case TOUCHSCREEN_CONFIG_MOVHYSTN:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.movhystn;
				else
					touchscreen_config.movhystn = arg;
				break;
			case TOUCHSCREEN_CONFIG_MOVFILTER:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.movfilter;
				else
					touchscreen_config.movfilter = arg;
				break;
			case TOUCHSCREEN_CONFIG_NUMTOUCH:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.numtouch;
				else
					touchscreen_config.numtouch= arg;
				break;
			case TOUCHSCREEN_CONFIG_MRGHYST:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.mrghyst;
				else
					touchscreen_config.mrghyst = arg;
				break;
			case TOUCHSCREEN_CONFIG_MRGTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.mrgthr ;
				else
					touchscreen_config.mrgthr = arg;
				break;
			case TOUCHSCREEN_CONFIG_AMPHYST:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.amphyst;
				else
					touchscreen_config.amphyst = arg;
				break;
			case TOUCHSCREEN_CONFIG_XRANGE:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xrange;
				else
					touchscreen_config.xrange = arg;
				break;
			case TOUCHSCREEN_CONFIG_YRANGE:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yrange;
				else
					touchscreen_config.yrange = arg;
				break;
			case TOUCHSCREEN_CONFIG_XLOCLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xloclip;
				else
					touchscreen_config.xloclip = arg;
				break;
			case TOUCHSCREEN_CONFIG_XHICLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xhiclip;
				else
					touchscreen_config.xhiclip = arg;
				break;
			case TOUCHSCREEN_CONFIG_YLOCLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yloclip;
				else
					touchscreen_config.yloclip =arg;
				break;
			case TOUCHSCREEN_CONFIG_YHICLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yhiclip;
				else
					touchscreen_config.yhiclip = arg;
				break;
			case TOUCHSCREEN_CONFIG_XEDGECTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xedgectrl;
				else
					touchscreen_config.xedgectrl = arg;
				break;
			case TOUCHSCREEN_CONFIG_XEDGEDIST:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xedgedist;
				else
					touchscreen_config.xedgedist = arg;
				break;
			case TOUCHSCREEN_CONFIG_YEDGECTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yedgectrl;
				else
					touchscreen_config.yedgectrl = arg;
				break;
			case TOUCHSCREEN_CONFIG_YEDGEDIST:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.yedgedist;
				else
					touchscreen_config.yedgedist = arg;
				break;
			case TOUCHSCREEN_CONFIG_JUMPLIMIT:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.jumplimit;
				else
					touchscreen_config.jumplimit    = arg;
				break;

			case TOUCHSCREEN_CONFIG_TCHHYST:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.tchhyst;
				else
					touchscreen_config.tchhyst    = arg;
				break;
			case TOUCHSCREEN_CONFIG_XPITCH:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.xpitch;
				else
					touchscreen_config.xpitch= arg;
				break;
			case TOUCHSCREEN_CONFIG_YPITCH:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.ypitch;
				else
					touchscreen_config.ypitch    = arg;
				break;
			case TOUCHSCREEN_CONFIG_NEXTTCHDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchscreen_config.nexttchdi;
				else
					touchscreen_config.nexttchdi= arg;
				break;

			case KEYARRAY_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.ctrl ;
				else
					keyarray_config.ctrl = arg;
				break;
			case KEYARRAY_CONFIG_XORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.xorigin;
				else
					keyarray_config.xorigin = arg;
				break;
			case KEYARRAY_CONFIG_YORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.yorigin;
				else
					keyarray_config.yorigin = arg;
				break;
			case KEYARRAY_CONFIG_XSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.xsize;
				else
					keyarray_config.xsize = arg;
				break;
			case KEYARRAY_CONFIG_YSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.ysize;
				else
					keyarray_config.ysize = arg;
				break;
			case KEYARRAY_CONFIG_AKSCFG:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.akscfg;
				else
					keyarray_config.akscfg = arg;
				break;
			case KEYARRAY_CONFIG_BLEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.blen;
				else
					keyarray_config.blen = arg;
				break;
			case KEYARRAY_CONFIG_TCHTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.tchthr;
				else
					keyarray_config.tchthr = arg;
				break;
			case KEYARRAY_CONFIG_TCHDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return keyarray_config.tchdi;
				else
					keyarray_config.tchdi = arg;
				break;

			case COMC_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return comc_config.ctrl;
				else
					comc_config.ctrl = arg;
				break;
			case COMC_CONFIG_COMMAND:
				if((int)arg == TOUCH_CONFIG_READ)
					return comc_config.cmd;
				else
					comc_config.cmd = arg;
				break;	

			case GPIOPWM_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.ctrl;
				else
					gpiopwm_config.ctrl = arg;
				break;
			case GPIOPWM_CONFIG_REPORTMASK:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.reportmask;
				else
					gpiopwm_config.reportmask = arg;
				break;
			case GPIOPWM_CONFIG_DIR:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.dir;
				else
					gpiopwm_config.dir = arg;
				break;
			case GPIOPWM_CONFIG_INTPULLUP:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.intpullup;
				else
					gpiopwm_config.intpullup = arg;
				break;
			case GPIOPWM_CONFIG_OUT:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.out;
				else
					gpiopwm_config.out = arg;
				break;
			case GPIOPWM_CONFIG_WAKE:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.wake;
				else
					gpiopwm_config.wake = arg;
				break;
			case GPIOPWM_CONFIG_PWM:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.pwm;
				else
					gpiopwm_config.pwm = arg;
				break;
			case GPIOPWM_CONFIG_PERIOD:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.period;
				else
					gpiopwm_config.period = arg;
				break;
			case GPIOPWM_CONFIG_DUTY0:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.duty[0];
				else
					gpiopwm_config.duty[0] = arg;
				break;
			case GPIOPWM_CONFIG_DUTY1:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.duty[1];
				else
					gpiopwm_config.duty[1] = arg;
				break;
			case GPIOPWM_CONFIG_DUTY2:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.duty[2];
				else
					gpiopwm_config.duty[2] = arg;
				break;
			case GPIOPWM_CONFIG_DUTY3:
				if((int)arg == TOUCH_CONFIG_READ)
					return gpiopwm_config.duty[3] ;
				else
					gpiopwm_config.duty[3] = arg;
				break;
			case GPIOPWM_CONFIG_TRIGGER0:
			case GPIOPWM_CONFIG_TRIGGER1:
			case GPIOPWM_CONFIG_TRIGGER2:
			case GPIOPWM_CONFIG_TRIGGER3:
				break;

			case GRIPFACESUPPRESSION_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return gripsuppression_t40_config.ctrl;
				else
					gripsuppression_t40_config.ctrl= arg;
				break;
			case GRIPFACESUPPRESSION_CONFIG_XLOGRIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return gripsuppression_t40_config.xlogrip;
				else
					gripsuppression_t40_config.xlogrip = arg;
				break;
			case GRIPFACESUPPRESSION_CONFIG_XHIGRIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return gripsuppression_t40_config.xhigrip;
				else
					gripsuppression_t40_config.xhigrip = arg;
				break;
			case GRIPFACESUPPRESSION_CONFIG_YLOGRIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return gripsuppression_t40_config.ylogrip;
				else
					gripsuppression_t40_config.ylogrip = arg;
				break;
			case GRIPFACESUPPRESSION_CONFIG_YHIGRIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return gripsuppression_t40_config.yhigrip;
				else
					gripsuppression_t40_config.yhigrip =arg;
				break;

			case TOUCHSUPPRESSION_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.ctrl;
				else
					touchsuppression_t42_config.ctrl =arg;
				break;
			case TOUCHSUPPRESSION_APPRTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.apprthr;
				else
					touchsuppression_t42_config.apprthr =arg;
				break;
			case TOUCHSUPPRESSION_MAXAPPRAREA:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.maxapprarea ;
				else
					touchsuppression_t42_config.maxapprarea =arg;
				break;
			case TOUCHSUPPRESSION_MAXTCHAREA:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.maxtcharea;
				else
					touchsuppression_t42_config.maxtcharea =arg;
				break;
			case TOUCHSUPPRESSION_SUPSTRENGTH:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.supstrength ;
				else
					touchsuppression_t42_config.supstrength =arg;
				break;
			case TOUCHSUPPRESSION_SUPEXTTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.supextto;
				else
					touchsuppression_t42_config.supextto =arg;
				break;
			case TOUCHSUPPRESSION_MAXNUMTCHS:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.maxnumtchs;
				else
					touchsuppression_t42_config.maxnumtchs =arg;
				break;
			case TOUCHSUPPRESSION_SHAPESTRENGTH:
				if((int)arg == TOUCH_CONFIG_READ)
					return touchsuppression_t42_config.shapestrength;
				else
					touchsuppression_t42_config.shapestrength =arg;
				break;		
			case NOISESUPPRESSION_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.ctrl ;
				else
					noisesuppression_t48_config.ctrl = arg;
				break;
			case NOISESUPPRESSION_CFG:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.cfg;
				else
					noisesuppression_t48_config.cfg = arg;
				break;
			case NOISESUPPRESSION_CALCFG:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.calcfg;
				else
					noisesuppression_t48_config.calcfg = arg;
				break;	
			case NOISESUPPRESSION_BASEFREQ:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.basefreq;
				else
					noisesuppression_t48_config.basefreq = arg;
				break;
			case NOISESUPPRESSION_MFFREQ0:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mffreq_2;
				else
					noisesuppression_t48_config.mffreq_2 = arg;
				break;
			case NOISESUPPRESSION_MFFREQ1:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mffreq_3;
				else
					noisesuppression_t48_config.mffreq_3 = arg;
				break;
			case NOISESUPPRESSION_GCACTVINVLDADCS:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.gcactvinvldadcs;
				else
					noisesuppression_t48_config.gcactvinvldadcs = arg;
				break;
			case NOISESUPPRESSION_GCIDLEINVLDADCS:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.gcidleinvldadcs;
				else
					noisesuppression_t48_config.gcidleinvldadcs = arg;
				break;
			case NOISESUPPRESSION_GCLIMITMIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.gclimitmin;
				else
					noisesuppression_t48_config.gclimitmin = arg;
				break;
			case NOISESUPPRESSION_GCLIMITMAX:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.gclimitmax;
				else
					noisesuppression_t48_config.gclimitmax = arg;
				break;
			case NOISESUPPRESSION_GCCOUNTMINTGT:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.gccountmintgt;
				else
					noisesuppression_t48_config.gccountmintgt = arg;
				break;
			case NOISESUPPRESSION_MFINVLDDIFFTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mfinvlddiffthr;
				else
					noisesuppression_t48_config.mfinvlddiffthr = arg;
				break;
			case NOISESUPPRESSION_MFINCADCSPXTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mfincadcspxthr;
				else
					noisesuppression_t48_config.mfincadcspxthr = arg;
				break;
			case NOISESUPPRESSION_MFERRORTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mferrorthr;
				else
					noisesuppression_t48_config.mferrorthr = arg;	
				break;
			case NOISESUPPRESSION_SELFREQMAX:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.selfreqmax;
				else
					noisesuppression_t48_config.selfreqmax = arg;
				break;
			case NOISESUPPRESSION_BLEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.blen;
				else
					noisesuppression_t48_config.blen = arg;
				break;
			case NOISESUPPRESSION_TCHTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.tchthr;
				else
					noisesuppression_t48_config.tchthr = arg;
				break;
			case NOISESUPPRESSION_TCHDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.tchdi;
				else
					noisesuppression_t48_config.tchdi = arg;
				break;
			case NOISESUPPRESSION_MOVHYSTI:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.movhysti;
				else
					noisesuppression_t48_config.movhysti= arg;
				break;
			case NOISESUPPRESSION_MOVHYSTN:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.movhystn ;
				else
					noisesuppression_t48_config.movhystn = arg;
				break;
			case NOISESUPPRESSION_MOVFILTER:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.movfilter;
				else
					noisesuppression_t48_config.movfilter = arg;	
				break;
			case NOISESUPPRESSION_NUMTOUCH://
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.numtouch;
				else
					noisesuppression_t48_config.numtouch = arg;
				break;
			case NOISESUPPRESSION_MRGHYST:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mrghyst;
				else
					noisesuppression_t48_config.mrghyst= arg;
				break;
			case NOISESUPPRESSION_MRGTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.mrgthr ;
				else
					noisesuppression_t48_config.mrgthr = arg;
				break;
			case NOISESUPPRESSION_XLOCLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.xloclip;
				else
					noisesuppression_t48_config.xloclip = arg;	
				break;
			case NOISESUPPRESSION_XHICLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.xhiclip;
				else
					noisesuppression_t48_config.xhiclip = arg;
				break;
			case NOISESUPPRESSION_YLOCLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.yloclip;
				else
					noisesuppression_t48_config.yloclip = arg;
				break;
			case NOISESUPPRESSION_YHICLIP:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.yhiclip;
				else
					noisesuppression_t48_config.yhiclip = arg;
				break;
			case NOISESUPPRESSION_XEDGECTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.xedgectrl;
				else
					noisesuppression_t48_config.xedgectrl = arg;
				break;
			case NOISESUPPRESSION_XEDGEDIST:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.xedgedist;
				else
					noisesuppression_t48_config.xedgedist = arg;
				break;
			case NOISESUPPRESSION_YEDGECTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.yedgectrl ;
				else
					noisesuppression_t48_config.yedgectrl = arg;
				break;
			case NOISESUPPRESSION_YEDGEDIST:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.yedgedist;
				else
					noisesuppression_t48_config.yedgedist = arg;	
				break;
			case NOISESUPPRESSION_JUMPLIMIT://
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.jumplimit;
				else
					noisesuppression_t48_config.jumplimit = arg;
				break;
			case NOISESUPPRESSION_TCHHYST:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.tchhyst;
				else				
					noisesuppression_t48_config.tchhyst= arg;
				break;
			case NOISESUPPRESSION_NEXTTCHDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return noisesuppression_t48_config.nexttchdi;
				else				
					noisesuppression_t48_config.nexttchdi = arg;	
				break;

			case STYLUS_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.ctrl ;
				else
					stylus_t47_config.ctrl = arg;	
				break;
			case STYLUS_CONTMIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.contmin;
				else
					stylus_t47_config.contmin = arg;	
				break;
			case STYLUS_CONTMAX:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.contmax;
				else
					stylus_t47_config.contmax = arg;	
				break;
			case STYLUS_STABILITY:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.stability;
				else
					stylus_t47_config.stability = arg;	
				break;
			case STYLUS_MAXTCHAREA:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.maxtcharea ;
				else
					stylus_t47_config.maxtcharea = arg;	
				break;
			case STYLUS_AMPLTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.amplthr;
				else
					stylus_t47_config.amplthr = arg;	
				break;
			case STYLUS_STYSHAPE:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.styshape;
				else
					stylus_t47_config.styshape = arg;	
				break;
			case STYLUS_HOVERSUP:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.hoversup;
				else
					stylus_t47_config.hoversup = arg;	
				break;
			case STYLUS_CONFTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.confthr;
				else
					stylus_t47_config.confthr = arg;	
				break;
			case STYLUS_SYNCSPERX:
				if((int)arg == TOUCH_CONFIG_READ)
					return stylus_t47_config.syncsperx;
				else
					stylus_t47_config.syncsperx = arg;	
				break;
			case PROXIMITY_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.ctrl ;
				else
					proximity_config.ctrl = arg;
				break;
			case PROXIMITY_CONFIG_XORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.xorigin;
				else
					proximity_config.xorigin =arg;
				break;
			case PROXIMITY_CONFIG_YORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.yorigin;
				else
					proximity_config.yorigin =arg;
				break;
			case PROXIMITY_CONFIG_XSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.xsize;
				else
					proximity_config.xsize = arg;
				break;
			case PROXIMITY_CONFIG_YSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.ysize;
				else
					proximity_config.ysize = arg;
				break;
			case PROXIMITY_CONFIG_BLEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.blen;
				else
					proximity_config.blen = arg;
				break;
			case PROXIMITY_CONFIG_FXDDTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.fxddthr;
				else
					proximity_config.fxddthr = arg; 
				break;
			case PROXIMITY_CONFIG_FXDDI:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.fxddi;
				else
					proximity_config.fxddi = arg; 
				break;
			case PROXIMITY_CONFIG_AVERAGE:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.average;
				else
					proximity_config.average = arg;     
				break;
			case PROXIMITY_CONFIG_MVNULLRATE:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.mvnullrate;
				else
					proximity_config.mvnullrate = arg; 
				break;
			case PROXIMITY_CONFIG_MVDTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return proximity_config.mvdthr;
				else
					proximity_config.mvdthr = arg; 
				break;
	#if 0
			case PROXIMITY_CONFIG_TCHTHR:
				proximity_config.tchthr = arg;
				break;
			case PROXIMITY_CONFIG_TCHDI:
				proximity_config.tchdi = arg;
				break;
			case PROXIMITY_CONFIG_AVERAGE:
				proximity_config.average = arg;
				break;
			case PROXIMITY_CONFIG_RATE:
				proximity_config.rate =arg;
				break;        
	#endif    

			case ONETOUCH_GESTURE_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.ctrl;
				else
					onetouch_gesture_config.ctrl = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_NUMGEST:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.numgest ;
				else
					onetouch_gesture_config.numgest = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_GESTEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.gesten ;
				else
					onetouch_gesture_config.gesten = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_PRESSPROC:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.pressproc;
				else
					onetouch_gesture_config.pressproc = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_TAPTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.tapto;
				else
					onetouch_gesture_config.tapto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_FLICKTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.flickto;
				else
					onetouch_gesture_config.flickto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_DRAGTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.dragto;
				else
					onetouch_gesture_config.dragto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_SPRESSTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.spressto;
				else
					onetouch_gesture_config.spressto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_LPRESSTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.lpressto;
				else
					onetouch_gesture_config.lpressto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_REPPRESSTO:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.reppressto;
				else
					onetouch_gesture_config.reppressto = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_FLICKTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.flickthr ;
				else
					onetouch_gesture_config.flickthr = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_DRAGTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.dragthr ;
				else
					onetouch_gesture_config.dragthr = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_TAPTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.tapthr;
				else
					onetouch_gesture_config.tapthr = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_THROWTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return onetouch_gesture_config.throwthr;
				else
					onetouch_gesture_config.throwthr = arg;
				break;
			case ONETOUCH_GESTURE_CONFIG_PROCESS:
				break;

			case SELFTEST_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return selftest_config.ctrl;
				else
					selftest_config.ctrl = arg;
				break;
			case SELFTEST_CONFIG_CMD:
				if((int)arg == TOUCH_CONFIG_READ)
					return selftest_config.cmd;
				else				
					selftest_config.cmd = arg;
				break;

			case TWOTOUCH_GESTURE_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return twotouch_gesture_config.ctrl;
				else				
					twotouch_gesture_config.ctrl = arg;
				break;
			case TWOTOUCH_GESTURE_CONFIG_NUMGEST:
				if((int)arg == TOUCH_CONFIG_READ)
					return twotouch_gesture_config.numgest;
				else
					twotouch_gesture_config.numgest = arg;
				break;
			case TWOTOUCH_GESTURE_CONFIG_GESTEN:
				if((int)arg == TOUCH_CONFIG_READ)
					return twotouch_gesture_config.gesten;
				else
					twotouch_gesture_config.gesten = arg;
				break;
			case TWOTOUCH_GESTURE_CONFIG_ROTATETHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return twotouch_gesture_config.rotatethr;
				else
					twotouch_gesture_config.rotatethr = arg;
				break;
			case TWOTOUCH_GESTURE_CONFIG_ZOOMTHR:
				if((int)arg == TOUCH_CONFIG_READ)
					return twotouch_gesture_config.zoomthr;
				else
					twotouch_gesture_config.zoomthr = arg;
				break;

			case CTE_CONFIG_CTRL:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.ctrl;
				else
					cte_t46_config.ctrl = arg;
				break;
			case CTE_CONFIG_MODE:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.mode;
				else
					cte_t46_config.mode = arg;
				break;
			case CTE_CONFIG_IDLEGCAFDEPTH:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.idlesyncsperx;
				else
					cte_t46_config.idlesyncsperx = arg;
				break;
			case CTE_CONFIG_ACTVGCAFDEPTH:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.actvsyncsperx;
				else
					cte_t46_config.actvsyncsperx = arg;
				break;
			case CTE_CONFIG_ADCSPERSYNC:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.adcspersync;
				else
					cte_t46_config.adcspersync = arg;
				break;
			case CTE_CONFIG_PULSESPERADC:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.pulsesperadc;
				else
					cte_t46_config.pulsesperadc = arg;
				break;
			case CTE_CONFIG_XSLEW:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.xslew;
				else
					cte_t46_config.xslew = arg;
				break;
			case CTE_CONFIG_SYNCDELAY:
				if((int)arg == TOUCH_CONFIG_READ)
					return cte_t46_config.syncdelay;
				else
					cte_t46_config.syncdelay = arg;
				break;

			case GENDATASOURCE_DSTYPE:
				if((int)arg == TOUCH_CONFIG_READ)
					return gendatasource_t53_config.dstype;
				else
					gendatasource_t53_config.dstype = arg;
				break;
			case GENDATASOURCE_XORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return gendatasource_t53_config.xorigin;
				else
					gendatasource_t53_config.xorigin  = arg;
				break;
			case GENDATASOURCE_XSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return gendatasource_t53_config.xsize;
				else
					gendatasource_t53_config.xsize = arg;
				break;
			case GENDATASOURCE_YORIGIN:
				if((int)arg == TOUCH_CONFIG_READ)
					return gendatasource_t53_config.yorigin;
				else
					gendatasource_t53_config.yorigin = arg;
				break;			
			case GENDATASOURCE_YSIZE:
				if((int)arg == TOUCH_CONFIG_READ)
					return gendatasource_t53_config.ysize;
				else
					gendatasource_t53_config.ysize = arg;
				break;
			
				//apply touch config	
			case APPLY_TOUCH_CONFIG:
				apply_touch_config();
				break;

				//Factory Cmd
			case TOUCH_IOCTL_READ_LASTKEY:
				break;
			case TOUCH_IOCTL_DO_KEY:
				input_report_key(qt602240_data->input_dev, (int)arg, 1);
				break;
			case TOUCH_IOCTL_RELEASE_KEY:		
				input_report_key(qt602240_data->input_dev, (int)arg, 0);
				break;
			case TOUCH_IOCTL_INIT:
				printk("Touch init \n");
				qt602240_front_test_init();
				break;
			case TOUCH_IOCTL_OFF:
				printk("Touch off \n");
				off_hw_setting();
				break;
			case DIAGNOSTIC_REFERENCE_DEBUG:
			case DIAGNOSTIC_DELTA_DEBUG:
				{
					int* send_byte;

					if(cmd == DIAGNOSTIC_REFERENCE_DEBUG)
					{
						send_byte = pantech_diagnostic_debug(QT602240_REFERENCE_MODE);
						diagnostic_min = QT602240_REFERENCE_MIN;
						diagnostic_max = QT602240_REFERENCE_MAX;
					}
					else if(cmd == DIAGNOSTIC_DELTA_DEBUG)
						send_byte = pantech_diagnostic_debug(QT602240_DELTA_MODE);
			
					if (copy_to_user(argp, send_byte, sizeof(int) * QT602240_MAX_CHANNEL_NUM))
	                            	return false;	

					return touch_diagnostic_ret;
					break;
				}	
			case DIAGNOSTIC_MIN_DEBUG:
				return diagnostic_min;	
				break;
			case DIAGNOSTIC_MAX_DEBUG:
				return diagnostic_max;
				break;				
			default:
				break;
		}

	return 0;
}

static void apply_touch_config(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	if (write_power_config(power_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg("xxxx acquisition config\n");
	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg("xxxx tchthr : %d",  touchscreen_config.tchthr);
	dbg("xxxx multitouchscreen config\n");
	if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg("xxxx keyarray config \n");
	if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg("xxxx comc config \n");
	if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
	{
		if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg("xxxx gpio pwm config \n");
	if (write_gpio_config(0, gpiopwm_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}	

	/*
	dbg("xxxx gripface suppression config \n");
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_config(0, gripfacesuppression_config) !=CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	*/

	/*
	dbg("xxxx noise suppression config \n");
	if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND)
	{
		if (write_noisesuppression_config(0,noise_suppression_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	*/
	
	dbg("xxxx proximity config \n");
	if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND)
	{
		if (write_proximity_config(0, proximity_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

/*
	dbg("xxxx one touch gesture config \n");
	if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) !=OBJECT_NOT_FOUND)
	{
		if (write_onetouchgesture_config(0, onetouch_gesture_config) !=CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
*/
	dbg("xxxx self test config \n");
	if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND)
	{
		if (write_selftest_config(0,selftest_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

#ifndef __VER_2_0__
/*
	dbg("xxxx two touch config \n");
	if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) !=OBJECT_NOT_FOUND)
	{
		if (write_twotouchgesture_config(0, twotouch_gesture_config) !=CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
*/
#endif
#if 0
#if defined(__VER_2_0__)
	if(info_block->info_id.version < 0x20)  //V2.0
#endif
	{
		dbg("xxxx two touch config \n");
		if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) !=OBJECT_NOT_FOUND)
		{
			if (write_twotouchgesture_config(0, twotouch_gesture_config) !=CFG_WRITE_OK)
			{
				dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
			}
		}
	}
#endif        

	if (get_object_address(PROCI_GRIPSUPPRESSION_T40, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_T40_config(gripsuppression_t40_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	
	if (get_object_address(PROCI_TOUCHSUPPRESSION_T42, 0) != OBJECT_NOT_FOUND)
	{
		if (write_touchsuppression_t42_config(touchsuppression_t42_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	
	if (get_object_address(SPT_CTECONFIG_T46, 0) != OBJECT_NOT_FOUND)
	{
		if (write_CTE_T46_config(cte_t46_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	
	if (get_object_address(PROCI_STYLUS_T47, 0) != OBJECT_NOT_FOUND)
	{
		if (write_stylus_t47_config(stylus_t47_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
	
	if (get_object_address(PROCG_NOISESUPPRESSION_T48, 0) != OBJECT_NOT_FOUND)
	{
		if (write_noisesuppression_t48_config(noisesuppression_t48_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	if (get_object_address(GENDATASOURCE_T53, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gendatasource_t53_config(gendatasource_t53_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}

static void pantech_stylus_mode_config(int mode)
{
	printk("[TSP] pantech_stylus_mode_config [charger_mode=%d, pre_charger_mode=%d], stylus mode %d \n", 
		(int)mode, (int)pre_charger_mode, stylus_mode);

	if(stylus_mode == true)
	{
		switch (mode)
		{
			case BATTERY_PLUGGED_NONE:	
				touchscreen_config.tchthr =  STYLUS_UNPLUGGED_T9_TCHTHR;
				touchscreen_config.movhysti=  STYLUS_UNPLUGGED_T9_MOVHYSTI;
				touchscreen_config.movhystn = STYLUS_UNPLUGGED_T9_MOVHYSTN;
				touchscreen_config.movfilter = STYLUS_UNPLUGGED_T9_MOVFILTER;
				touchscreen_config.jumplimit = STYLUS_UNPLUGGED_T9_JUMPLIMIT;

				touchsuppression_t42_config.ctrl = STYLUS_UNPLUGGED_T42_CTRL;
				
				stylus_t47_config.ctrl = STYLUS_UNPLUGGED_T47_CTRL;
				stylus_t47_config.amplthr = STYLUS_UNPLUGGED_T47_AMPLTHR;

				noisesuppression_t48_config.ctrl = STYLUS_UNPLUGGED_T48_CTRL;
				noisesuppression_t48_config.calcfg = STYLUS_UNPLUGGED_T48_CALCFG;
				noisesuppression_t48_config.cfg = STYLUS_UNPLUGGED_T48_CFG;
				noisesuppression_t48_config.mferrorthr = STYLUS_UNPLUGGED_T48_MFERRORTHR;
				noisesuppression_t48_config.selfreqmax = STYLUS_UNPLUGGED_T48_SELFREQMAX;
				noisesuppression_t48_config.blen = STYLUS_UNPLUGGED_T48_BLEN;
				noisesuppression_t48_config.tchthr = STYLUS_UNPLUGGED_T48_TCHTHR;
				noisesuppression_t48_config.movhysti = STYLUS_UNPLUGGED_T48_MOVHYSTI;
				noisesuppression_t48_config.movhystn = STYLUS_UNPLUGGED_T48_MOVHYSTN;
				noisesuppression_t48_config.mrgthr = STYLUS_UNPLUGGED_T48_MRGTHR;
				noisesuppression_t48_config.xloclip = STYLUS_UNPLUGGED_T48_XLOCLIP;
				noisesuppression_t48_config.xhiclip = STYLUS_UNPLUGGED_T48_XHICLIP;
				noisesuppression_t48_config.yloclip = STYLUS_UNPLUGGED_T48_YLOCLIP;
				noisesuppression_t48_config.yhiclip = STYLUS_UNPLUGGED_T48_YHICLIP;
				noisesuppression_t48_config.yedgedist = STYLUS_UNPLUGGED_T48_YEDGEDIST;
				noisesuppression_t48_config.jumplimit = STYLUS_UNPLUGGED_T48_JUMPLIMIT;
				noisesuppression_t48_config.nexttchdi = STYLUS_UNPLUGGED_T48_NEXTTCHDI;
				break;
				
			case BATTERY_PLUGGED_AC:
			case BATTERY_PLUGGED_USB:
				touchscreen_config.tchthr =  STYLUS_PLUGGED_T9_TCHTHR;
				touchscreen_config.movhysti=  STYLUS_PLUGGED_T9_MOVHYSTI;
				touchscreen_config.movhystn = STYLUS_PLUGGED_T9_MOVHYSTN;
				touchscreen_config.movfilter = STYLUS_PLUGGED_T9_MOVFILTER;
				touchscreen_config.jumplimit = STYLUS_PLUGGED_T9_JUMPLIMIT;

				touchsuppression_t42_config.ctrl = STYLUS_PLUGGED_T42_CTRL;
				
				stylus_t47_config.ctrl = STYLUS_PLUGGED_T47_CTRL;
				stylus_t47_config.amplthr = STYLUS_PLUGGED_T47_AMPLTHR;

				noisesuppression_t48_config.ctrl = STYLUS_PLUGGED_T48_CTRL;
				noisesuppression_t48_config.calcfg = STYLUS_PLUGGED_T48_CALCFG;
				noisesuppression_t48_config.cfg = STYLUS_PLUGGED_T48_CFG;
				noisesuppression_t48_config.mferrorthr = STYLUS_PLUGGED_T48_MFERRORTHR;
				noisesuppression_t48_config.selfreqmax = STYLUS_PLUGGED_T48_SELFREQMAX;
				noisesuppression_t48_config.blen = STYLUS_PLUGGED_T48_BLEN;
				noisesuppression_t48_config.tchthr = STYLUS_PLUGGED_T48_TCHTHR;
				noisesuppression_t48_config.movhysti = STYLUS_PLUGGED_T48_MOVHYSTI;
				noisesuppression_t48_config.movhystn = STYLUS_PLUGGED_T48_MOVHYSTN;
				noisesuppression_t48_config.mrgthr = STYLUS_PLUGGED_T48_MRGTHR;
				noisesuppression_t48_config.xloclip = STYLUS_PLUGGED_T48_XLOCLIP;
				noisesuppression_t48_config.xhiclip = STYLUS_PLUGGED_T48_XHICLIP;
				noisesuppression_t48_config.yloclip = STYLUS_PLUGGED_T48_YLOCLIP;
				noisesuppression_t48_config.yhiclip = STYLUS_PLUGGED_T48_YHICLIP;
				noisesuppression_t48_config.yedgedist = STYLUS_PLUGGED_T48_YEDGEDIST;
				noisesuppression_t48_config.jumplimit = STYLUS_PLUGGED_T48_JUMPLIMIT;
				noisesuppression_t48_config.nexttchdi = STYLUS_PLUGGED_T48_NEXTTCHDI;				
				break;
			default:
				break;			
		}
	}
	else
	{
		switch (mode)
		{
			case BATTERY_PLUGGED_NONE:	
				touchscreen_config.tchthr =  NORMAL_UNPLUGGED_T9_TCHTHR;
				touchscreen_config.movhysti=  NORMAL_UNPLUGGED_T9_MOVHYSTI;
				touchscreen_config.movhystn = NORMAL_UNPLUGGED_T9_MOVHYSTN;
				touchscreen_config.movfilter = NORMAL_UNPLUGGED_T9_MOVFILTER;
				touchscreen_config.jumplimit = NORMAL_UNPLUGGED_T9_JUMPLIMIT;

				touchsuppression_t42_config.ctrl = NORMAL_UNPLUGGED_T42_CTRL;
				
				stylus_t47_config.ctrl = NORMAL_UNPLUGGED_T47_CTRL;
				stylus_t47_config.amplthr = NORMAL_UNPLUGGED_T47_AMPLTHR;

				noisesuppression_t48_config.ctrl = NORMAL_UNPLUGGED_T48_CTRL;
				noisesuppression_t48_config.calcfg = NORMAL_UNPLUGGED_T48_CALCFG;
				noisesuppression_t48_config.cfg = NORMAL_UNPLUGGED_T48_CFG;
				noisesuppression_t48_config.mferrorthr = NORMAL_UNPLUGGED_T48_MFERRORTHR;
				noisesuppression_t48_config.selfreqmax = NORMAL_UNPLUGGED_T48_SELFREQMAX;
				noisesuppression_t48_config.blen = NORMAL_UNPLUGGED_T48_BLEN;
				noisesuppression_t48_config.tchthr = NORMAL_UNPLUGGED_T48_TCHTHR;
				noisesuppression_t48_config.movhysti = NORMAL_UNPLUGGED_T48_MOVHYSTI;
				noisesuppression_t48_config.movhystn = NORMAL_UNPLUGGED_T48_MOVHYSTN;
				noisesuppression_t48_config.mrgthr = NORMAL_UNPLUGGED_T48_MRGTHR;
				noisesuppression_t48_config.xloclip = NORMAL_UNPLUGGED_T48_XLOCLIP;
				noisesuppression_t48_config.xhiclip = NORMAL_UNPLUGGED_T48_XHICLIP;
				noisesuppression_t48_config.yloclip = NORMAL_UNPLUGGED_T48_YLOCLIP;
				noisesuppression_t48_config.yhiclip = NORMAL_UNPLUGGED_T48_YHICLIP;
				noisesuppression_t48_config.yedgedist = NORMAL_UNPLUGGED_T48_YEDGEDIST;
				noisesuppression_t48_config.jumplimit = NORMAL_UNPLUGGED_T48_JUMPLIMIT;
				noisesuppression_t48_config.nexttchdi = NORMAL_UNPLUGGED_T48_NEXTTCHDI;
				break;
				
			case BATTERY_PLUGGED_AC:
			case BATTERY_PLUGGED_USB:
				touchscreen_config.tchthr =  NORMAL_PLUGGED_T9_TCHTHR;
				touchscreen_config.movhysti=  NORMAL_PLUGGED_T9_MOVHYSTI;
				touchscreen_config.movhystn = NORMAL_PLUGGED_T9_MOVHYSTN;
				touchscreen_config.movfilter = NORMAL_PLUGGED_T9_MOVFILTER;
				touchscreen_config.jumplimit = NORMAL_PLUGGED_T9_JUMPLIMIT;

				touchsuppression_t42_config.ctrl = NORMAL_PLUGGED_T42_CTRL;
				
				stylus_t47_config.ctrl = NORMAL_PLUGGED_T47_CTRL;
				stylus_t47_config.amplthr = NORMAL_PLUGGED_T47_AMPLTHR;

				noisesuppression_t48_config.ctrl = NORMAL_PLUGGED_T48_CTRL;
				noisesuppression_t48_config.calcfg = NORMAL_PLUGGED_T48_CALCFG;
				noisesuppression_t48_config.cfg = NORMAL_PLUGGED_T48_CFG;
				noisesuppression_t48_config.mferrorthr = NORMAL_PLUGGED_T48_MFERRORTHR;
				noisesuppression_t48_config.selfreqmax = NORMAL_PLUGGED_T48_SELFREQMAX;
				noisesuppression_t48_config.blen = NORMAL_PLUGGED_T48_BLEN;
				noisesuppression_t48_config.tchthr = NORMAL_PLUGGED_T48_TCHTHR;
				noisesuppression_t48_config.movhysti = NORMAL_PLUGGED_T48_MOVHYSTI;
				noisesuppression_t48_config.movhystn = NORMAL_PLUGGED_T48_MOVHYSTN;
				noisesuppression_t48_config.mrgthr = NORMAL_PLUGGED_T48_MRGTHR;
				noisesuppression_t48_config.xloclip = NORMAL_PLUGGED_T48_XLOCLIP;
				noisesuppression_t48_config.xhiclip = NORMAL_PLUGGED_T48_XHICLIP;
				noisesuppression_t48_config.yloclip = NORMAL_PLUGGED_T48_YLOCLIP;
				noisesuppression_t48_config.yhiclip = NORMAL_PLUGGED_T48_YHICLIP;
				noisesuppression_t48_config.yedgedist = NORMAL_PLUGGED_T48_YEDGEDIST;
				noisesuppression_t48_config.jumplimit = NORMAL_PLUGGED_T48_JUMPLIMIT;
				noisesuppression_t48_config.nexttchdi = NORMAL_PLUGGED_T48_NEXTTCHDI;				
				break;
			default:
				break;			
		}	
	}

	mutex_lock(&qt602240_data->lock);

	if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
	{
		printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	if (write_touchsuppression_t42_config(touchsuppression_t42_config) != CFG_WRITE_OK)
	{
		printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	if (write_stylus_t47_config(stylus_t47_config) != CFG_WRITE_OK)
	{
		printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	if (write_noisesuppression_t48_config(noisesuppression_t48_config) != CFG_WRITE_OK)
	{
		printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}	

	mutex_unlock(&qt602240_data->lock);

	//
	// Must Delete!!!!!!
	//
	//backup_config();

	return;
	
}


#ifdef CHARGER_MODE
static uint8_t qt_charger_mode_config(unsigned long mode)
{
	uint8_t status = 0;
	uint16_t object_address_t7, object_address_t48= 0;
	//uint16_t object_address_t28, object_address_t7,object_address_t22 = 0;

	dbg_func_in();

	printk("[TSP] qt_charger_mode_config [charger_mode=%d, pre_charger_mode=%d], MXT768? %d Stylus mode: %d, charger_mode_flag : %d\n", 
		(int)mode, (int)pre_charger_mode, IS_QT602240_MXT768E(tsp_family_id), stylus_mode, charger_mode_flag);

	if (driver_setup != DRIVER_SETUP_OK)
		return 0;
/*
	if(pre_charger_mode == mode || charger_mode_flag == false)
		return status;
	
	pre_charger_mode = mode;
*/
	if(stylus_mode == true)
	{
		pantech_stylus_mode_config(mode);
		return status;
	}

	switch (mode)
	{
		case BATTERY_PLUGGED_NONE:
			printk("Touch charger mode BATTERY_PLUGGED_NONE \n");
			if(IS_QT602240_MXT224E(tsp_family_id) || IS_QT602240_MXT768E(tsp_family_id))
			{
				noisesuppression_t48_config.calcfg = T48_CALCFG;
				power_config.idleacqint = T7_IDLEACQINT;
			}
		
			break;
			
		case BATTERY_PLUGGED_AC:
		case BATTERY_PLUGGED_USB:
			printk("Touch charger mode BATTERY_PLUGGED \n");
			if(IS_QT602240_MXT224E(tsp_family_id) || IS_QT602240_MXT768E(tsp_family_id)){				
				noisesuppression_t48_config.calcfg = T48_CALCFG_PLUG;
				dbg("noisesuppression_t48_config.calcfg = %d\n",noisesuppression_t48_config.calcfg);
				power_config.idleacqint = T7_IDLEACQINT_PLUG;
			}

			break;
		default:
			break;
	}


/*
	object_address_t9 = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, 0);
	dbg("object_address_t9 = %d\n",object_address_t9);
	if (object_address_t9 == 0)
		return(CFG_WRITE_FAILED);
*/

	object_address_t7 = get_object_address(GEN_POWERCONFIG_T7, 0);
	dbg("object_address_t7 = %d\n",object_address_t7);
	if (object_address_t7 == 0)
		return(CFG_WRITE_FAILED);

	object_address_t48 = get_object_address(PROCG_NOISESUPPRESSION_T48, 0);
	dbg("object_address_t48 = %d\n",object_address_t48);
	if (object_address_t48 == 0)
		return(CFG_WRITE_FAILED);

/*
	if(!(IS_QT602240_MXT224E(tsp_family_id)))
	{
		object_address_t7 = get_object_address(GEN_POWERCONFIG_T7, 0);
		dbg("object_address_t7 = %d\n",object_address_t7);
		if (object_address_t7 == 0)
			return(CFG_WRITE_FAILED);
		
		object_address_t22 = get_object_address(PROCG_NOISESUPPRESSION_T22,	0);
		dbg("object_address_t22 = %d\n",object_address_t22);
		if (object_address_t22 == 0)
			return(CFG_WRITE_FAILED);

		object_address_t28 = get_object_address(SPT_CTECONFIG_T28, 0);
		dbg("object_address_t28 = %d\n",object_address_t28);
		if (object_address_t28 == 0)
			return(CFG_WRITE_FAILED);	
	}
*/


	mutex_lock(&qt602240_data->lock);

/*
	if(!(IS_QT602240_MXT224E(tsp_family_id)))
	{
		status |= write_mem(object_address_t22+8, 1, &noise_suppression_config.noisethr);
		status |= write_mem(object_address_t28+3, 1, &cte_config.idlegcafdepth);
		status |= write_mem(object_address_t28+4, 1, &cte_config.actvgcafdepth);
		status |= write_mem(object_address_t7, 1, &power_config.idleacqint);
		status |= write_mem(object_address_t7+1, 1, &power_config.actvacqint);		
	}
*/
	//dbg("[TSP] CHRGER MODE SET [%d]  \n", charger_mode);
/*
		status |= write_mem(object_address_t9+7, 1, &touchscreen_config.tchthr);
		status |= write_mem(object_address_t9+8, 1, &touchscreen_config.tchdi);
#if defined(__VER_2_0__)	
		status |= write_mem(object_address_t9+31, 1, &touchscreen_config.tchhyst);
#endif
*/

	status |= write_mem(object_address_t7, 1, &power_config.idleacqint);

	if (write_noisesuppression_t48_config(noisesuppression_t48_config) != CFG_WRITE_OK)
	{
		printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg("noisesuppression_t48_config.calcfg = %d\n",noisesuppression_t48_config.calcfg);
		
	if(mode == BATTERY_PLUGGED_NONE)
	{
		if(noise_setting_flag == true)
		{
			del_timer(&timer_chk_fhe);
			if(fherr_during_noise_timer == true)
				clear_event(TSC_CLEAR_ALL);

			noise_setting_flag = false;
			timer_tick_fhe_elapse = false;
			fherr_during_noise_timer = false;
		}     
	}

	mutex_unlock(&qt602240_data->lock);

	dbg_func_out();
	return status;
}

#endif // CHARGER_MODE
#endif //TOUCH_IO

/*****************************************************************************
 *
 *       QT602240  Configuration Block
 *
 * ***************************************************************************/

void qt_Power_Sleep(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	/* Set Idle Acquisition Interval to 32 ms. */
	power_config.idleacqint = 0;

	/* Set Active Acquisition Interval to 16 ms. */
	power_config.actvacqint = 0;

	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	power_config.actv2idleto = 0;

	/* Write power config to chip. */
	if (write_power_config(power_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}

/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Power_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	if(IS_QT602240_MXT224E(tsp_family_id)){
		power_config.idleacqint = T7_IDLEACQINT;
		power_config.actvacqint = T7_ACTVACQINT;//255
		power_config.actv2idleto = T7_ACTV2IDLETO;
	}else{
		power_config.idleacqint = T7_IDLEACQINT;
		/* Set Active Acquisition Interval to 16 ms. */
		power_config.actvacqint = T7_ACTVACQINT;//255
		/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
		power_config.actv2idleto = T7_ACTV2IDLETO;
	}

	/* Write power config to chip. */
	if (write_power_config(power_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}




/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Acquisition_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	acquisition_config.chrgtime = T8_CHRGTIME; // 2us
	acquisition_config.reserved = 0;	//5;
	acquisition_config.tchdrift = T8_TCHDRIFT;	 // 4s
	acquisition_config.driftst = T8_DRIFTST;	 // 4s
	acquisition_config.tchautocal = T8_TCHAUTOCAL; // infinite
	acquisition_config.sync = T8_SYNC; // disabled
	acquisition_config.atchcalst = T8_ATCHCALST;
	acquisition_config.atchcalsthr = T8_ATCHCALSTHR;
	acquisition_config.atchfrccalthr = T8_ATCHFRCCALTHR;     /*!< Anti-touch force calibration threshold */   
	acquisition_config.atchfrccalration = T8_ATCHFRCCALRATIO;  /*!< Anti-touch force calibration ratio */  

	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Multitouchscreen_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	//		touchscreen_config.ctrl = 143/*0x8f*/; //131; // enable + message-enable
	touchscreen_config.ctrl = T9_CTRL; //131; // enable + message-enable
	touchscreen_config.movhysti = T9_MOVHYSTI/*10*/;	// Relate Touch Move and Click
	touchscreen_config.movhystn = T9_MOVHYSTN/*2*/;
	touchscreen_config.mrghyst = T9_MRGHYST/*10*/;
	touchscreen_config.mrgthr = T9_MRGTHR/*30*/;
	touchscreen_config.amphyst = T9_AMPHYST/*10*/;
	touchscreen_config.yloclip = T9_YLOCLIP/*15*/;	// Change Active area
	touchscreen_config.yhiclip = T9_YHICLIP/*15*/;
	touchscreen_config.jumplimit = T9_JUMPLIMIT/*30*/;
	touchscreen_config.xpitch = T9_XPITCH;
	touchscreen_config.ypitch =  T9_YPITCH;
	touchscreen_config.nexttchdi =  T9_NEXTTCHDI;
	touchscreen_config.blen = T9_BLEN;
	touchscreen_config.tchthr = T9_TCHTHR;
	touchscreen_config.movfilter = T9_MOVFILTER;
	touchscreen_config.tchdi = T9_TCHDI;

	touchscreen_config.tchhyst = T9_TCHHYST;   // 25% of tchthr
	touchscreen_config.xorigin = T9_XORIGIN;
	touchscreen_config.yorigin = T9_YORIGIN;
	touchscreen_config.xsize = T9_XSIZE;
	touchscreen_config.ysize = T9_YSIZE;

	touchscreen_config.akscfg = T9_AKSCFG;

	touchscreen_config.orient = T9_ORIENT;

	touchscreen_config.mrgtimeout = T9_MRGTIMEOUT;

	touchscreen_config.movfilter = T9_MOVFILTER;
	touchscreen_config.numtouch= T9_NUMTOUCH;

	touchscreen_config.xrange = T9_XRANGE;    // 902 = (98.2/87.1) * 800
	touchscreen_config.yrange = T9_YRANGE;
	touchscreen_config.xloclip = T9_XLOCLIP;
	touchscreen_config.xhiclip = T9_XHICLIP;

	touchscreen_config.xedgectrl = T9_XEDGECTRL;
	touchscreen_config.xedgedist = T9_XEDGEDIST;
	touchscreen_config.yedgectrl = T9_YEDGECTRL;
	touchscreen_config.yedgedist = T9_YEDGEDIST;

	if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_KeyArray_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	keyarray_config.ctrl = T15_CTRL;
	keyarray_config.xorigin = T15_XORIGIN;
	keyarray_config.yorigin = T15_YORIGIN;
	keyarray_config.xsize = T15_XSIZE;
	keyarray_config.ysize = T15_YSIZE;
	keyarray_config.akscfg = T15_AKSCFG;
	keyarray_config.blen = T15_BLEN;
	keyarray_config.tchthr = T15_TCHTHR;
	keyarray_config.tchdi = T15_TCHDI;
	keyarray_config.reserved[0] = T15_RESERVED_0;
	keyarray_config.reserved[1] = T15_RESERVED_1;

	if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_ComcConfig_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	comc_config.ctrl = T18_CTRL;
	if(IS_QT602240_MXT224E(tsp_family_id)){
		comc_config.cmd = NO_COMMAND;
	}else{
		comc_config.cmd = T18_COMMAND;
	}

	if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
	{
		if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Gpio_Pwm_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	gpiopwm_config.ctrl = T19_CTRL;
	gpiopwm_config.reportmask = T19_REPORTMASK;
	gpiopwm_config.dir = T19_DIR;
	gpiopwm_config.intpullup = T19_INTPULLUP;
	gpiopwm_config.out = T19_OUT;
	gpiopwm_config.wake = T19_WAKE;
	gpiopwm_config.pwm = T19_PWM;
	gpiopwm_config.period = T19_PERIOD;
	gpiopwm_config.duty[0] = T19_DUTY_0;
	gpiopwm_config.duty[1] = T19_DUTY_1;
	gpiopwm_config.duty[2] = T19_DUTY_2;
	gpiopwm_config.duty[3] = T19_DUTY_3;
	gpiopwm_config.trigger[0] = T19_TRIGGER_0;
	gpiopwm_config.trigger[1] = T19_TRIGGER_1;
	gpiopwm_config.trigger[2] = T19_TRIGGER_2;
	gpiopwm_config.trigger[3] = T19_TRIGGER_3;

	if (write_gpio_config(0, gpiopwm_config) != CFG_WRITE_OK)
	{
		dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Grip_Face_Suppression_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	gripfacesuppression_config.ctrl = T20_CTRL;
	gripfacesuppression_config.xlogrip = T20_XLOGRIP;
	gripfacesuppression_config.xhigrip = T20_XHIGRIP;
	gripfacesuppression_config.ylogrip = T20_YLOGRIP;
	gripfacesuppression_config.yhigrip = T20_YHIGRIP;
	gripfacesuppression_config.maxtchs = T20_MAXTCHS;
	gripfacesuppression_config.reserved = T20_RESERVED_0;
	gripfacesuppression_config.szthr1 = T20_SZTHR1;
	gripfacesuppression_config.szthr2 = T20_SZTHR2;
	gripfacesuppression_config.shpthr1 = T20_SHPTHR1;
	gripfacesuppression_config.shpthr2 = T20_SHPTHR2;
	gripfacesuppression_config.supextto = T20_SUPEXTTO;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_config(0, gripfacesuppression_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Noise_Suppression_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	noise_suppression_config.ctrl = T22_CTRL;
	noise_suppression_config.reserved = T22_RESERVED_0;
	noise_suppression_config.reserved1 = T22_RESERVED_1;
	noise_suppression_config.gcaful = T22_GCAFUL;
	noise_suppression_config.gcafll = -T22_GCAFLL;

	noise_suppression_config.actvgcafvalid = T22_ACTVGCAFVALID;
	noise_suppression_config.noisethr = T22_NOISETHR;
	noise_suppression_config.freqhopscale = T22_FREQHOPSCALE;

	noise_suppression_config.freq[0] = T22_FREQ_0;
	noise_suppression_config.freq[1] = T22_FREQ_1;
	noise_suppression_config.freq[2] = T22_FREQ_2;
	noise_suppression_config.freq[3] = T22_FREQ_3;
	noise_suppression_config.freq[4] = T22_FREQ_4;
	noise_suppression_config.idlegcafvalid = T22_IDLEGCAFVALID;

	/* Write Noise suppression config to chip. */
	if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND)
	{
		if (write_noisesuppression_config(0,noise_suppression_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Proximity_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	proximity_config.ctrl = T23_CTRL;
	proximity_config.xorigin = T23_XORIGIN;
	proximity_config.yorigin = T23_YORIGIN;    
	proximity_config.xsize = T23_XSIZE;
	proximity_config.ysize = T23_YSIZE;
	proximity_config.reserved_for_future_aks_usage = T23_RESERVED;
	proximity_config.blen = T23_BLEN;
	proximity_config.fxddthr = T23_FXDDTHR; 
	proximity_config.fxddi = T23_FXDDI; 
	proximity_config.average = T23_AVERAGE;     
	proximity_config.mvnullrate = T23_MVNULLRATE; 
	proximity_config.mvdthr = T23_MVDTHR; 

	if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND)
	{
		if (write_proximity_config(0, proximity_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_One_Touch_Gesture_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	/* Disable one touch gestures. */
	onetouch_gesture_config.ctrl = T24_CTRL;
	onetouch_gesture_config.numgest = T24_NUMGEST;

	onetouch_gesture_config.gesten = T24_GESTEN;
	onetouch_gesture_config.pressproc = T24_PROCESS;
	onetouch_gesture_config.tapto = T24_TAPTO;
	onetouch_gesture_config.flickto = T24_FLICKTO;
	onetouch_gesture_config.dragto = T24_DRAGTO;
	onetouch_gesture_config.spressto = T24_SPRESSTO;
	onetouch_gesture_config.lpressto = T24_LPRESSTO;
	onetouch_gesture_config.reppressto = T24_REPPRESSTO;
	onetouch_gesture_config.flickthr = T24_FLICKTHR;
	onetouch_gesture_config.dragthr = T24_DRAGTHR;
	onetouch_gesture_config.tapthr = T24_TAPTHR;
	onetouch_gesture_config.throwthr = T24_THROWTHR;

	if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) !=
			OBJECT_NOT_FOUND)
	{
		if (write_onetouchgesture_config(0, onetouch_gesture_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Selftest_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	selftest_config.ctrl = T25_CTRL;
	selftest_config.cmd = T25_CMD;

#if(NUM_OF_TOUCH_OBJECTS)

	if(IS_QT602240_MXT224E(tsp_family_id)){
		selftest_config.siglim[0].upsiglim= 0;
		selftest_config.siglim[0].losiglim = 0;

		selftest_config.siglim[1].upsiglim = 0;
		selftest_config.siglim[1].losiglim= 0;

		selftest_config.siglim[2].upsiglim = 0;
		selftest_config.siglim[2].losiglim = 0;
	}else{
		siglim.upsiglim[0] = T25_SIGLIM_0_UPSIGLIM;
		siglim.losiglim[0] = T25_SIGLIM_0_LOSIGLIM;
	}

#endif
	if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND)
	{
		if (write_selftest_config(0,selftest_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_Two_touch_Gesture_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	/* Disable two touch gestures. */
	twotouch_gesture_config.ctrl = T27_CTRL;
	twotouch_gesture_config.numgest = T27_NUMGEST;
	twotouch_gesture_config.reserved2 = T27_RESERVED_0;
	twotouch_gesture_config.gesten = T27_GESTEN;
	twotouch_gesture_config.rotatethr = T27_ROTATETHR;
	twotouch_gesture_config.zoomthr = T27_ZOOMTHR;

	if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) !=
			OBJECT_NOT_FOUND)
	{
		if (write_twotouchgesture_config(0, twotouch_gesture_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void qt_CTE_Config_Init(void)
{
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	/* Set CTE config */
	cte_config.ctrl = T28_CTRL;
	cte_config.cmd = T28_CMD; 
	cte_config.mode = T28_MODE;
	cte_config.idlegcafdepth = T28_IDLEGCAFDEPTH;	//T28_IDLEGCAFDEPTH;
	cte_config.actvgcafdepth = T28_ACTVGCAFDEPTH;
	if(tsp_version >= 0x15)
	{
		cte_config.voltage = T28_VOLTAGE; //0x1E; //(AVDD-2.7)/0.01 , 3V-2.7V /0.01 = 30;
	}

	/* Write CTE config to chip. */
	if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND)
	{
		if (write_CTE_config(cte_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();
}

void qt_Grip_Face_Suppression_Config_On(void)
{
	if (driver_setup != DRIVER_SETUP_OK)
		return;

	dbg_func_in();

	gripfacesuppression_config.ctrl = 7; 

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_config(0, gripfacesuppression_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();

}

void qt_Grip_Face_Suppression_Config_Off(void)
{
	if (driver_setup != DRIVER_SETUP_OK)
		return;

	dbg_func_in();

	gripfacesuppression_config.ctrl = 0;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_config(0, gripfacesuppression_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}

	dbg_func_out();

}
#if defined(__MXT224E_CONFIG__)

/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * 2011-01-19 10:29:00
 * MXT224E 
 *
 * ***************************************************************************/

void qt_Grip_Suppression_T40_Config_Init(void)
{
	gripsuppression_t40_config.ctrl     = T40_CTRL;
	gripsuppression_t40_config.xlogrip  = T40_XLOGRIP;
	gripsuppression_t40_config.xhigrip  = T40_XHIGRIP;
	gripsuppression_t40_config.ylogrip  = T40_YLOGRIP;
	gripsuppression_t40_config.yhigrip  = T40_YHIGRIP;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPSUPPRESSION_T40, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_T40_config(gripsuppression_t40_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}


void qt_Proci_palmsuppression_T41_config_Init(void)
{
#ifndef __MXT224E_CONFIG__
	palmsuppression_t41_config.ctrl         = T41_CTRL;
	palmsuppression_t41_config.reserved_0   = T41_RESERVED_0;
	palmsuppression_t41_config.reserved_1   = T41_RESERVED_1;
	palmsuppression_t41_config.largeobjthr  = T41_LARGEOBJTHR;
	palmsuppression_t41_config.distancethr  = T41_DISTANCETHR;
	palmsuppression_t41_config.supextto     = T41_SUPEXTTO;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_PALMSUPPRESSION_T41, 0) != OBJECT_NOT_FOUND)
	{
		if (write_palmsuppression_T41_config(palmsuppression_t41_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
#endif
}


void qt_Touch_Suppression_T42_Config_Init(void)
{
	touchsuppression_t42_config.ctrl             = T42_CTRL;         
	touchsuppression_t42_config.apprthr          = T42_APPRTHR;      
	touchsuppression_t42_config.maxapprarea      = T42_MAXAPPRAREA;  
	touchsuppression_t42_config.maxtcharea       = T42_MAXTCHAREA;   
	touchsuppression_t42_config.supstrength      = T42_SUPSTRENGTH;  
	touchsuppression_t42_config.supextto         = T42_SUPEXTTO;     
	touchsuppression_t42_config.maxnumtchs       = T42_MAXNUMTCHS;   
	touchsuppression_t42_config.shapestrength    = T42_SHAPESTRENGTH;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_TOUCHSUPPRESSION_T42, 0) != OBJECT_NOT_FOUND)
	{
		if (write_touchsuppression_t42_config(touchsuppression_t42_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}


void qt_DIGITIZER_T43_Config_Init(void)
{
#ifndef __MXT224E_CONFIG__
	digitizer_t43_config.ctrl               = T42_CTRL;         
	digitizer_t43_config.hididlerate        = T43_HIDIDLERATE;      
	digitizer_t43_config.xlength      	= T43_XLENGTH;  
	digitizer_t43_config.ylength            = T43_YLENGTH;   

	/* Write spt digitizer t43 config to chip. */
	if (get_object_address(SPT_DIGITIZER_T43, 0) != OBJECT_NOT_FOUND)
	{
		if (write_digitizer_t43_config(digitizer_t43_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
#endif
}


void qt_CTE_T46_Config_Init(void)
{
	/* Set CTE config */
	cte_t46_config.ctrl                 = T46_CTRL;
	cte_t46_config.mode                 = T46_MODE;
	cte_t46_config.idlesyncsperx        = T46_IDLESYNCSPERX;
	cte_t46_config.actvsyncsperx        = T46_ACTVSYNCSPERX;
	cte_t46_config.adcspersync          = T46_ADCSPERSYNC;
	cte_t46_config.pulsesperadc         = T46_PULSESPERADC;
	cte_t46_config.xslew                = T46_XSLEW;
	cte_t46_config.syncdelay            = T46_SYNCDELAY;

	/* Write CTE config to chip. */
	if (get_object_address(SPT_CTECONFIG_T46, 0) != OBJECT_NOT_FOUND)
	{
		if (write_CTE_T46_config(cte_t46_config) != CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}


void qt_Stylus_T47_Config_Init(void)
{
	stylus_t47_config.ctrl          = T47_CTRL; 
	stylus_t47_config.contmin       = T47_CONTMIN; 
	stylus_t47_config.contmax       = T47_CONTMAX; 
	stylus_t47_config.stability     = T47_STABILITY; 
	stylus_t47_config.maxtcharea    = T47_MAXTCHAREA;  
	stylus_t47_config.amplthr       = T47_AMPLTHR; 
	stylus_t47_config.styshape      = T47_STYSHAPE; 
	stylus_t47_config.hoversup      = T47_HOVERSUP; 
	stylus_t47_config.confthr       = T47_CONFTHR; 
	stylus_t47_config.syncsperx     = T47_SYNCSPERX;


	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_STYLUS_T47, 0) != OBJECT_NOT_FOUND)
	{
		if (write_stylus_t47_config(stylus_t47_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}


void qt_Noisesuppression_T48_config_Init(void)
{
	noisesuppression_t48_config.ctrl  			= T48_CTRL ;	
	noisesuppression_t48_config.cfg  			= T48_CFG ;
	noisesuppression_t48_config.calcfg  			= T48_CALCFG ;
	noisesuppression_t48_config.basefreq  			= T48_BASEFREQ ;
	noisesuppression_t48_config.freq_0  			= T48_RESERVED0 ;
	noisesuppression_t48_config.freq_1  			= T48_RESERVED1 ;
	noisesuppression_t48_config.freq_2  			= T48_RESERVED2 ;
	noisesuppression_t48_config.freq_3  			= T48_RESERVED3 ;
	noisesuppression_t48_config.mffreq_2  			= T48_MFFREQ_2 ;
	noisesuppression_t48_config.mffreq_3  			= T48_MFFREQ_3 ;
	noisesuppression_t48_config.nlgain  			= T48_RESERVED4 ;
	noisesuppression_t48_config.nlthr  			= T48_RESERVED5 ;
	noisesuppression_t48_config.gclimit  			= T48_RESERVED6 ;
	noisesuppression_t48_config.gcactvinvldadcs  		= T48_GCACTVINVLDADCS ;
	noisesuppression_t48_config.gcidleinvldadcs  		= T48_GCIDLEINVLDADCS ;
	noisesuppression_t48_config.gcinvalidthr  		= T48_RESERVED7 ;
	/* noisesuppression_t48_config.reserved8  		= T48_RESERVED8 ; */
	noisesuppression_t48_config.gcmaxadcsperx  		= T48_GCMAXADCSPERX ;
	noisesuppression_t48_config.gclimitmin  		= T48_GCLIMITMIN ;
	noisesuppression_t48_config.gclimitmax  		= T48_GCLIMITMAX ;
	noisesuppression_t48_config.gccountmintgt  		= T48_GCCOUNTMINTGT ;
	noisesuppression_t48_config.mfinvlddiffthr  		= T48_MFINVLDDIFFTHR ;
	noisesuppression_t48_config.mfincadcspxthr  		= T48_MFINCADCSPXTHR ;
	noisesuppression_t48_config.mferrorthr  		= T48_MFERRORTHR ;
	noisesuppression_t48_config.selfreqmax  		= T48_SELFREQMAX ;
	noisesuppression_t48_config.reserved9  			= T48_RESERVED9 ;
	noisesuppression_t48_config.reserved10  		= T48_RESERVED10 ;
	noisesuppression_t48_config.reserved11  		= T48_RESERVED11 ;
	noisesuppression_t48_config.reserved12  		= T48_RESERVED12 ;
	noisesuppression_t48_config.reserved13  		= T48_RESERVED13 ;
	noisesuppression_t48_config.reserved14 		 	= T48_RESERVED14 ;
	noisesuppression_t48_config.blen  			= T48_BLEN ;
	noisesuppression_t48_config.tchthr  			= T48_TCHTHR ;
	noisesuppression_t48_config.tchdi  			= T48_TCHDI ;
	noisesuppression_t48_config.movhysti  			= T48_MOVHYSTI ;
	noisesuppression_t48_config.movhystn  			= T48_MOVHYSTN ;
	noisesuppression_t48_config.movfilter  			= T48_MOVFILTER ;
	noisesuppression_t48_config.numtouch  			= T48_NUMTOUCH ;
	noisesuppression_t48_config.mrghyst  			= T48_MRGHYST ;
	noisesuppression_t48_config.mrgthr  			= T48_MRGTHR ;
	noisesuppression_t48_config.xloclip  			= T48_XLOCLIP ;
	noisesuppression_t48_config.xhiclip  			= T48_XHICLIP ;
	noisesuppression_t48_config.yloclip  			= T48_YLOCLIP ;
	noisesuppression_t48_config.yhiclip  			= T48_YHICLIP ;
	noisesuppression_t48_config.xedgectrl  			= T48_XEDGECTRL ;
	noisesuppression_t48_config.xedgedist  			= T48_XEDGEDIST ;
	noisesuppression_t48_config.yedgectrl  			= T48_YEDGECTRL ;
	noisesuppression_t48_config.yedgedist  			= T48_YEDGEDIST ;
	noisesuppression_t48_config.jumplimit  			= T48_JUMPLIMIT ;
	noisesuppression_t48_config.tchhyst  			= T48_TCHHYST ;
	noisesuppression_t48_config.nexttchdi  			= T48_NEXTTCHDI ;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCG_NOISESUPPRESSION_T48, 0) != OBJECT_NOT_FOUND)
	{
		if (write_noisesuppression_t48_config(noisesuppression_t48_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}

#endif // MXT224E

#if 1//def __MXT768E_CONFIG__

void qt_Digitizer_T43_Config_Init(void)
{
	spt_digitizer_t43_config.ctrl  			= T43_CTRL ;	
	spt_digitizer_t43_config.hididlerate		= T43_HIDIDLERATE;
	spt_digitizer_t43_config.xlength		= T43_XLENGTH;
	spt_digitizer_t43_config.ylength		= T43_YLENGTH;
	spt_digitizer_t43_config.rwkrate		= T43_RWKRATE;

	/* Write grip suppression config to chip. */
	if (get_object_address(SPT_DIGITIZER_T43, 0) != OBJECT_NOT_FOUND)
	{
		if (write_digitizer_t43_config(spt_digitizer_t43_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}

void qt_Proximitykey_T52_Config_Init(void)
{
	proximitykey_t52_config.ctrl  			= T52_CTRL;	
	proximitykey_t52_config.xorigin		= T52_XORIGIN;
	proximitykey_t52_config.yorigin		= T52_YORIGIN;
	proximitykey_t52_config.reserved0	= T52_RESERVED0;
	proximitykey_t52_config.reserved1	= T52_RESERVED1;
	proximitykey_t52_config.akscfg		= T52_AKSCFG;
	proximitykey_t52_config.reserved2	= T52_RESERVED2;
	proximitykey_t52_config.fxddthr		= T52_FXDDTHR;
	proximitykey_t52_config.fxddi			= T52_FXDDI;
	proximitykey_t52_config.average		= T52_AVERAGE;
	proximitykey_t52_config.mvnullrate	= T52_MVNULLRATE;
	proximitykey_t52_config.mvdthr		= T52_MVDTHR;

	/* Write grip suppression config to chip. */
	if (get_object_address(PROXIMITYKEY_T52, 0) != OBJECT_NOT_FOUND)
	{
		if (write_proximitykey_t52_config(proximitykey_t52_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}

void qt_Gendatasource_T53_Config_Init(void)
{
	gendatasource_t53_config.dstype  			= T53_DSTYPE;
	gendatasource_t53_config.xorigin			= T53_XORIGIN;
	gendatasource_t53_config.xsize			= T53_XSIZE;
	gendatasource_t53_config.yorigin			= T53_YORIGIN;
	gendatasource_t53_config.ysize			= T53_YSIZE;

	/* Write grip suppression config to chip. */
	if (get_object_address(GENDATASOURCE_T53, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gendatasource_t53_config(gendatasource_t53_config) !=
				CFG_WRITE_OK)
		{
			dbg("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
		}
	}
}


#endif

/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

unsigned char Comm_Config_Process(unsigned char change_en)
{
	dbg_func_in();

	change_en = 0;

	dbg_func_out();

	return change_en;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t reset_chip(void)
{
	uint8_t data = 1u;
	uint8_t rc;

	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return WRITE_MEM_FAILED;

	rc = write_mem(command_processor_address + RESET_OFFSET, 1, &data);

	dbg_func_out();

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 * \brief Calibrates the chip.
 * 
 * This function will send a calibrate command to touch chip.
 * Whilst calibration has not been confirmed as good, this function will set
 * the ATCHCALST and ATCHCALSTHR to zero to allow a bad cal to always recover
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t calibrate_chip(void)
{
	uint8_t data = 1u;
	int ret = WRITE_MEM_OK;
//	uint8_t atchcalst, atchcalsthr;

	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return WRITE_MEM_FAILED;

	/* change calibration suspend settings to zero until calibration confirmed good */
	/* store normal settings */
//	atchcalst = acquisition_config.atchcalst;
//	atchcalsthr = acquisition_config.atchcalsthr;

	/* resume calibration must be performed with zero settings */
//	acquisition_config.atchcalst = 0;
//	acquisition_config.atchcalsthr = 0; 

	dbg("[TSP] calibrate_chip acq atchcalst=%d, atchcalsthr=%d\n", acquisition_config.atchcalst, acquisition_config.atchcalsthr );

	/* Write temporary acquisition config to chip. */
	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
	{
		/* "Acquisition config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
		ret = WRITE_MEM_FAILED; /* calling function should retry calibration call */
	}

	/* restore settings to the local structure so that when we confirm the 
	 * cal is good we can correct them in the chip */
	/* this must be done before returning */
//	acquisition_config.atchcalst = atchcalst;
//	acquisition_config.atchcalsthr = atchcalsthr;


	/* send calibration command to the chip */
	if(ret == WRITE_MEM_OK)
	{
		/* change calibration suspend settings to zero until calibration confirmed good */
		ret = write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data);

		/* set flag for calibration lockup recovery if cal command was successful */
		if(ret == WRITE_MEM_OK)
		{ 
			/* set flag to show we must still confirm if calibration was good or bad */
			cal_check_flag = 1u;
		}
	}

	msleep(120);

	dbg_func_out();
	return ret;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t diagnostic_chip(uint8_t mode)
{
	uint8_t status;
	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return WRITE_MEM_FAILED;

	status = write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &mode);

	dbg_func_out();
	return(status);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t backup_config(void)
{
	/* Write 0x55 to BACKUPNV register to initiate the backup. */
	uint8_t data = 0x55u;
	uint8_t rc;

	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return WRITE_MEM_FAILED;

	rc = write_mem(command_processor_address + BACKUP_OFFSET, 1, &data);

	dbg_func_out();

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t get_version(uint8_t *version)
{
	dbg_func_in();

	if (info_block)
	{
		*version = info_block->info_id.version;
	}
	else
	{
		return(ID_DATA_NOT_AVAILABLE);
	}

	dbg_func_out();

	return (ID_DATA_OK);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t get_family_id(void)
{
	uint8_t rc = 0;

	dbg_func_in();
	
	read_mem(0, 1, (void *) &tsp_family_id);	

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t get_build_number(uint8_t *build)
{
	uint8_t rc;

	dbg_func_in();

	if (info_block)
	{
		*build = info_block->info_id.build;
		rc = ID_DATA_OK;
	}
	else
	{
		rc = ID_DATA_NOT_AVAILABLE;
	}

	dbg_func_out();

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t get_variant_id(uint8_t *variant)
{
	uint8_t rc;

	dbg_func_in();

	if (info_block)
	{
		*variant = info_block->info_id.variant_id;
		rc = ID_DATA_OK;
	}
	else
	{
		rc = ID_DATA_NOT_AVAILABLE;
	}

	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_power_config(gen_powerconfig_t7_config_t cfg)
{
	uint8_t rc;

	dbg_func_in();

	rc = write_simple_config(GEN_POWERCONFIG_T7, 0, (void *) &cfg);

	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg)
{
	uint8_t rc;

	dbg_func_in();

	rc = write_simple_config(GEN_ACQUISITIONCONFIG_T8, 0, (void *) &cfg);

	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_multitouchscreen_config(uint8_t instance, touch_multitouchscreen_t9_config_t cfg)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);


	/* 18 elements at beginning are 1 byte. */
	memcpy(tmp, &cfg, 18);

	/* Next two are 2 bytes. */

	*(tmp + 18) = (uint8_t) (cfg.xrange &  0xFF);
	*(tmp + 19) = (uint8_t) (cfg.xrange >> 8);

	*(tmp + 20) = (uint8_t) (cfg.yrange &  0xFF);
	*(tmp + 21) = (uint8_t) (cfg.yrange >> 8);

	/* And the last 4(8) 1 bytes each again. */

	*(tmp + 22) = cfg.xloclip;
	*(tmp + 23) = cfg.xhiclip;
	*(tmp + 24) = cfg.yloclip;
	*(tmp + 25) = cfg.yhiclip;

	*(tmp + 26) = cfg.xedgectrl;
	*(tmp + 27) = cfg.xedgedist;
	*(tmp + 28) = cfg.yedgectrl;
	*(tmp + 29) = cfg.yedgedist;

	//if(info_block->info_id.version >= 0x16)
	//{
		*(tmp + 30) = cfg.jumplimit;
	//}

	*(tmp + 31) = cfg.tchhyst;

	if(IS_QT602240_MXT224E(tsp_family_id)){
		*(tmp + 32) = cfg.xpitch;
		*(tmp + 33) = cfg.ypitch;
		*(tmp + 34) = cfg.nexttchdi;
	}

	object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);

	dbg_func_out();
	return(status);

}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_keyarray_config(uint8_t instance, touch_keyarray_t15_config_t cfg)
{
	uint8_t rc;
	dbg_func_in();

	rc = write_simple_config(TOUCH_KEYARRAY_T15, instance, (void *) &cfg);

	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg)
{
	uint8_t rc;
	dbg_func_in();

	rc = write_simple_config(SPT_COMCONFIG_T18, instance, (void *) &cfg);

	dbg_func_out();
	return rc;
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg)
{
	uint8_t rc;
	dbg_func_in();
	rc = write_simple_config(SPT_GPIOPWM_T19, instance, (void *) &cfg);
	dbg_func_out();
	return rc;
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg)
{
	uint8_t rc;
	dbg_func_in();
	rc = write_simple_config(PROCI_GRIPFACESUPPRESSION_T20, instance, (void *) &cfg);
	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(PROCG_NOISESUPPRESSION_T22);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);


	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.reserved;
	*(tmp + 2) = cfg.reserved1;

	*(tmp + 3) = (uint8_t) (cfg.gcaful & 0x00FF);
	*(tmp + 4) = (uint8_t) (cfg.gcaful >> 8);

	*(tmp + 5) = (uint8_t) (cfg.gcafll & 0x00FF);
	*(tmp + 6) = (uint8_t) (cfg.gcafll >> 8);

	*(tmp + 7) = cfg.actvgcafvalid;
	*(tmp + 8) = cfg.noisethr;

	*(tmp + 9) = cfg.reserved2;
	*(tmp + 10) = cfg.freqhopscale;

	*(tmp + 11) = cfg.freq[0];
	*(tmp + 12) = cfg.freq[1];
	*(tmp + 13) = cfg.freq[2];
	*(tmp + 14) = cfg.freq[3];
	*(tmp + 15) = cfg.freq[4];

	*(tmp + 16) = cfg.idlegcafvalid;


	object_address = get_object_address(PROCG_NOISESUPPRESSION_T22,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);
	dbg_func_out();
	return(status);
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(TOUCH_PROXIMITY_T23);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.xorigin;
	*(tmp + 2) = cfg.yorigin;
	*(tmp + 3) = cfg.xsize;
	*(tmp + 4) = cfg.ysize;
	*(tmp + 5) = cfg.reserved_for_future_aks_usage;
	*(tmp + 6) = cfg.blen;

	*(tmp + 7) = (uint8_t) (cfg.fxddthr & 0x00FF);
	*(tmp + 8) = (uint8_t) (cfg.fxddthr >> 8);

	*(tmp + 9) = cfg.fxddi;
	*(tmp + 10) = cfg.average;

	*(tmp + 11) = (uint8_t) (cfg.mvnullrate & 0x00FF);
	*(tmp + 12) = (uint8_t) (cfg.mvnullrate >> 8);

	*(tmp + 13) = (uint8_t) (cfg.mvdthr & 0x00FF);
	*(tmp + 14) = (uint8_t) (cfg.mvdthr >> 8);
#if 0
	*(tmp + 7) = (uint8_t) (cfg.tchthr & 0x00FF);
	*(tmp + 8) = (uint8_t) (cfg.tchthr >> 8);

	*(tmp + 9) = cfg.tchdi;
	*(tmp + 10) = cfg.average;

	*(tmp + 11) = (uint8_t) (cfg.rate & 0x00FF);
	*(tmp + 12) = (uint8_t) (cfg.rate >> 8);
#endif

	object_address = get_object_address(TOUCH_PROXIMITY_T23,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);
	dbg_func_out();
	return(status);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.numgest;

	*(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
	*(tmp + 3) = (uint8_t) (cfg.gesten >> 8);

	memcpy((tmp+4), &cfg.pressproc, 7);

	*(tmp + 11) = (uint8_t) (cfg.flickthr & 0x00FF);
	*(tmp + 12) = (uint8_t) (cfg.flickthr >> 8);

	*(tmp + 13) = (uint8_t) (cfg.dragthr & 0x00FF);
	*(tmp + 14) = (uint8_t) (cfg.dragthr >> 8);

	*(tmp + 15) = (uint8_t) (cfg.tapthr & 0x00FF);
	*(tmp + 16) = (uint8_t) (cfg.tapthr >> 8);

	*(tmp + 17) = (uint8_t) (cfg.throwthr & 0x00FF);
	*(tmp + 18) = (uint8_t) (cfg.throwthr >> 8);

	object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);
	kfree(tmp);
	dbg_func_out();
	return(status);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg)
{

	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(SPT_SELFTEST_T25);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);


	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.cmd;
#if(NUM_OF_TOUCH_OBJECTS)
	if(IS_QT602240_MXT224E(tsp_family_id)){
		*(tmp + 2) = (uint8_t) (cfg.siglim[0].upsiglim & 0x00FF);
		*(tmp + 3) = (uint8_t) (cfg.siglim[0].upsiglim >> 8);
		//   *(tmp + 2) = (uint8_t) (cfg.losiglim & 0x00FF);
		//   *(tmp + 3) = (uint8_t) (cfg.losiglim >> 8);
		*(tmp + 4) = (uint8_t) (cfg.siglim[1].upsiglim & 0x00FF);
		*(tmp + 5) = (uint8_t) (cfg.siglim[1].upsiglim >> 8);
		*(tmp + 6) = (uint8_t) (cfg.siglim[2].upsiglim & 0x00FF);
		*(tmp + 7) = (uint8_t) (cfg.siglim[2].upsiglim >> 8);
	}
	else
	{
		*(tmp + 2) = (uint8_t) (cfg.upsiglim & 0x00FF);
		*(tmp + 3) = (uint8_t) (cfg.upsiglim >> 8);
		*(tmp + 2) = (uint8_t) (cfg.losiglim & 0x00FF);
		*(tmp + 3) = (uint8_t) (cfg.losiglim >> 8);
	}

#endif
	object_address = get_object_address(SPT_SELFTEST_T25,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);
	dbg_func_out();
	return(status);
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg)
{

	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	dbg_func_in();

	object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.numgest;
	*(tmp + 2) = 0;
	*(tmp + 3) = cfg.gesten;
	*(tmp + 4) = cfg.rotatethr;
	*(tmp + 5) = (uint8_t) (cfg.zoomthr & 0x00FF);
	*(tmp + 6) = (uint8_t) (cfg.zoomthr >> 8);

	object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27,
			instance);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);
	kfree(tmp);
	dbg_func_out();
	return(status);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg)
{
	uint8_t rc;
	dbg_func_in();
	rc = write_simple_config(SPT_CTECONFIG_T28, 0, (void *) &cfg);
	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 *  MXT224E 
 *
 * ***************************************************************************/


uint8_t write_gripsuppression_T40_config(proci_gripsuppression_t40_config_t cfg)
{
	return(write_simple_config(PROCI_GRIPSUPPRESSION_T40, 0, (void *) &cfg));
}


uint8_t write_palmsuppression_T41_config(proci_palmsuppression_t41_config_t cfg)
{
	return(write_simple_config(PROCI_PALMSUPPRESSION_T41, 0, (void *) &cfg));
}


uint8_t write_touchsuppression_t42_config(proci_touchsuppression_t42_config_t cfg)
{
	return(write_simple_config(PROCI_TOUCHSUPPRESSION_T42, 0, (void *) &cfg));
}

uint8_t write_digitizer_t43_config(spt_digitizer_t43_config_t cfg)
{
	return(write_simple_config(SPT_DIGITIZER_T43, 0, (void *) &cfg));
}

uint8_t write_CTE_T46_config(spt_cteconfig_t46_config_t cfg)
{

	return(write_simple_config(SPT_CTECONFIG_T46, 0, (void *) &cfg));
}


uint8_t  write_stylus_t47_config(proci_stylus_t47_config_t cfg)
{
	return(write_simple_config(PROCI_STYLUS_T47, 0, (void *) &cfg));
}


uint8_t  write_noisesuppression_t48_config(procg_noisesuppression_t48_config_t cfg)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	object_size = get_object_size(PROCG_NOISESUPPRESSION_T48);
	if (object_size == 0)
	{
		return(CFG_WRITE_FAILED);
	}
	tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
	if (tmp == NULL)
	{
		return(CFG_WRITE_FAILED);
	}

	memset(tmp,0,object_size);


	/* 18 elements at beginning are 1 byte. */
	memcpy(tmp, &cfg, 15);

	/* Next two are 2 bytes. */

	*(tmp + 15) = (uint8_t) (cfg.gcinvalidthr &  0xFF);
	*(tmp + 16) = (uint8_t) (cfg.gcinvalidthr >> 8);
	*(tmp + 17) = cfg.gcmaxadcsperx;
	*(tmp + 18) = cfg.gclimitmin;
	*(tmp + 19) = cfg.gclimitmax;
	*(tmp + 20) = (uint8_t) (cfg.gccountmintgt &  0xFF);
	*(tmp + 21) = (uint8_t) (cfg.gccountmintgt >> 8);
	*(tmp + 22) = cfg.mfinvlddiffthr;
	*(tmp + 23) = (uint8_t) (cfg.mfincadcspxthr &  0xFF);
	*(tmp + 24) = (uint8_t) (cfg.mfincadcspxthr >> 8);
	*(tmp + 25) = (uint8_t) (cfg.mferrorthr &  0xFF);
	*(tmp + 26) = (uint8_t) (cfg.mferrorthr >> 8);
	*(tmp + 27) = cfg.selfreqmax;
	*(tmp + 28) = cfg.reserved9;
	*(tmp + 29) = cfg.reserved10;
	*(tmp + 30) = cfg.reserved11;
	*(tmp + 31) = cfg.reserved12;
	*(tmp + 32) = cfg.reserved13;
	*(tmp + 33) = cfg.reserved14;
	*(tmp + 34) = cfg.blen;
	*(tmp + 35) = cfg.tchthr; 
	*(tmp + 36) = cfg.tchdi;
	*(tmp + 37) = cfg.movhysti;
	*(tmp + 38) = cfg.movhystn;
	*(tmp + 39) = cfg.movfilter;
	*(tmp + 40) = cfg.numtouch;
	*(tmp + 41) = cfg.mrghyst;
	*(tmp + 42) = cfg.mrgthr;
	*(tmp + 43) = cfg.xloclip;
	*(tmp + 44) = cfg.xhiclip;
	*(tmp + 45) = cfg.yloclip;
	*(tmp + 46) = cfg.yhiclip;
	*(tmp + 47) = cfg.xedgectrl;
	*(tmp + 48) = cfg.xedgedist;
	*(tmp + 49) = cfg.yedgectrl;
	*(tmp + 50) = cfg.yedgedist;
	*(tmp + 51) = cfg.jumplimit;
	*(tmp + 52) = cfg.tchhyst; 
	*(tmp + 53) = cfg.nexttchdi;

	object_address = get_object_address(PROCG_NOISESUPPRESSION_T48, 0);

	if (object_address == 0)
	{
		return(CFG_WRITE_FAILED);
	}

	status = write_mem(object_address, object_size, tmp);
	kfree(tmp);
	return(status);
}

uint8_t  write_proximitykey_t52_config(proximitykey_t52_config_t cfg)
{
	return(write_simple_config(PROXIMITYKEY_T52, 0, (void *) &cfg));
}

uint8_t  write_gendatasource_t53_config(gendatasource_t53_config_t cfg)
{
	return(write_simple_config(GENDATASOURCE_T53, 0, (void *) &cfg));
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg)
{
	uint16_t object_address;
	uint8_t object_size;
	uint8_t rc;

	dbg_func_in();

	object_address = get_object_address(object_type, instance);
	object_size = get_object_size(object_type);

	if ((object_size == 0) || (object_address == 0))
	{
		rc = CFG_WRITE_FAILED;
	}
	else
	{
		rc = write_mem(object_address, object_size, cfg);
	}

	dbg_func_out();
	return rc; 
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t get_object_size(uint8_t object_type)
{
	uint8_t object_table_index = 0;
	uint8_t object_found = 0;
	uint16_t size = OBJECT_NOT_FOUND;
	object_t *object_table;
	object_t obj;

	dbg_func_in();

	if(info_block == NULL)		
		return 0;

	object_table = info_block->objects;
	while ((object_table_index < info_block->info_id.num_declared_objects) &&
			!object_found)
	{
		obj = object_table[object_table_index];
		/* Does object type match? */
		if (obj.object_type == object_type)
		{
			object_found = 1;
			size = obj.size + 1;
		}
		object_table_index++;
	}

	dbg_func_out();
	return(size);
}

/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t type_to_report_id(uint8_t object_type, uint8_t instance)
{

	uint8_t report_id = 1;
	int8_t report_id_found = 0;
	uint8_t rc;

	dbg_func_in();

	while((report_id <= max_report_id) && (report_id_found == 0))
	{
		if((report_id_map[report_id].object_type == object_type) &&
				(report_id_map[report_id].instance == instance))
		{
			report_id_found = 1;
		}
		else
		{
			report_id++;
		}
	}
	if (report_id_found)
	{
		rc = report_id;
	}
	else
	{
		rc = ID_MAPPING_FAILED;
	}

	dbg_func_out();

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance)
{
	uint8_t rc;

	dbg_func_in();

	if (report_id <= max_report_id)
	{
		*instance = report_id_map[report_id].instance;
		rc = report_id_map[report_id].object_type;
	}
	else
	{
		rc = ID_MAPPING_FAILED;
	}

	dbg_func_out();

	return rc;
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t read_id_block(info_id_t *id)
{
	uint8_t status;

	dbg_func_in();

	status = read_mem(0, 1, (void *) &id->family_id);
	if (status != READ_MEM_OK)
	{
#if 1 //#if EF30S_BDVER_GE(EF30S_PT10) 
#else
		return(status);
#endif
	}
	dbg("family_id = %d\n",id->family_id);
    tsp_family_id = id->family_id;

	status = read_mem(1, 1, (void *) &id->variant_id);
	if (status != READ_MEM_OK) goto read_id_block_exit;
	dbg("variant_id = %d\n",id->variant_id);

	status = read_mem(2, 1, (void *) &id->version);
	if (status != READ_MEM_OK) goto read_id_block_exit;
	dbg("version = %d\n",id->version);

	status = read_mem(3, 1, (void *) &id->build);
	if (status != READ_MEM_OK) goto read_id_block_exit;
	dbg("familybuild_id = %d\n",id->build);

	status = read_mem(4, 1, (void *) &id->matrix_x_size);
	if (status != READ_MEM_OK) goto read_id_block_exit;
	dbg("matrix_x_size = %d\n",id->matrix_x_size);

	status = read_mem(5, 1, (void *) &id->matrix_y_size);
	if (status != READ_MEM_OK) goto read_id_block_exit;
	dbg("matrix_y_size = %d\n",id->matrix_y_size);

	status = read_mem(6, 1, (void *) &id->num_declared_objects);
	dbg("declared_objects = %d\n",id->num_declared_objects);

	dbg_func_out();

	return status;

read_id_block_exit:

	dbg("error : read_id_block_exit\n");
	dbg_func_out();
	return status;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = OBJECT_NOT_FOUND;
	object_t *object_table;
	object_t obj;

	dbg_func_in();

	if(info_block == NULL)		
		return 0;

	object_table = info_block->objects;
	while ((object_table_index < info_block->info_id.num_declared_objects) &&
			!address_found)
	{
		obj = object_table[object_table_index];
		/* Does object type match? */
		if (obj.object_type == object_type)
		{

			address_found = 1;

			/* Are there enough instances defined in the FW? */
			if (obj.instances >= instance)
			{
				address = obj.i2c_address + (obj.size + 1) * instance;
			}
		}
		object_table_index++;
	}

	dbg_func_out();
	return(address);
}



/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint32_t get_stored_infoblock_crc()
{
	uint32_t crc;

	dbg_func_in();

	crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
	crc = crc | info_block->CRC;

	dbg_func_out();
	return(crc);
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{

	uint32_t crc = 0;
	uint16_t crc_area_size;
	uint8_t *mem;
	uint8_t i;
	uint8_t rc;

	uint8_t status;

	dbg_func_in();

	rc = CRC_CALCULATION_OK;

	/* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
	crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

	mem = (uint8_t *) kmalloc(crc_area_size, GFP_KERNEL | GFP_ATOMIC);
	if (mem == NULL)
	{
		rc = CRC_CALCULATION_FAILED;
		goto calculate_infoblock_crc_exit;
	}

	status = read_mem(0, crc_area_size, mem);
	if (status != READ_MEM_OK)
	{
		rc = CRC_CALCULATION_FAILED;
		goto calculate_infoblock_crc_exit;
	}

	i = 0;
	while (i < (crc_area_size - 1))
	{
		crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
		i += 2;
	}

	crc = CRC_24(crc, *(mem + i), 0);

	kfree(mem);

	/* Return only 24 bit CRC. */
	*crc_pointer = (crc & 0x00FFFFFF);

	dbg_func_out();
	return rc;

calculate_infoblock_crc_exit:
	dbg("error : calculate_infoblock_crc\n");
	dbg_func_out();
	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
	static const uint32_t crcpoly = 0x80001B;
	uint32_t result;
	uint16_t data_word;

	dbg_func_in();

	data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
	result = ((crc << 1u) ^ (uint32_t) data_word);

	if (result & 0x1000000)
	{
		result ^= crcpoly;
	}

	dbg_func_out();

	return(result);
}

void touch_data_init(void)
{
	int i = 0;
	dbg_func_in();

	for (i = 0; i<MAX_NUM_FINGER; i++ )
	{
		fingerInfo[i].mode = TSC_EVENT_NONE;
		fingerInfo[i].status = -1;
		fingerInfo[i].area = 0;
		keyInfo[i].update = false;
	}
	active_event = true;

	calibration_message = 0; 
	timer_tick = 0;
	timer_enabled = false;
	facesup_flag = 0;

	noise_setting_flag = false;
	timer_tick_fhe_elapse = false;
	fherr_during_noise_timer = false;

	is_cal_success = false;

	dbg_func_out();
}

/*------------------------------ main block -----------------------------------*/
void quantum_touch_probe(void)
{
	U8 report_id;
	U8 object_type, instance;
	uint32_t crc, stored_crc;
	U8 variant = 0, build = 0;
#ifdef CHIP_NOINIT
	uint16_t object_address;
	uint16_t xres, yres;
	U8 xysize[3],  status;
#endif

	dbg_func_in();

	if (init_touch_driver(qt602240_data->client->addr ) == DRIVER_SETUP_OK)
	{
		dbg("\n[TSP] Touch device found\n");
	}
	else
	{
		dbg("\n[TSP][ERROR] Touch device NOT found\n");
		return ;
	}

	/* Get and show the version information. */
	get_family_id();
	get_variant_id(&variant);
	get_version(&tsp_version);
	get_build_number(&build);

	dbg("Version:        0x%x\n\r", tsp_version);
	dbg("Family:         0x%x\n\r", tsp_family_id);
	dbg("Variant:        0x%x\n\r", variant);
	dbg("Build number:   0x%x\n\r", build);

	dbg("Matrix X size : %d\n\r", info_block->info_id.matrix_x_size);
	dbg("Matrix Y size : %d\n\r", info_block->info_id.matrix_y_size);

	if(calculate_infoblock_crc(&crc) != CRC_CALCULATION_OK)
	{
		dbg("Calculating CRC failed, skipping check!\n\r");
	}
	else
	{
		dbg("Calculated CRC:\t");
		write_message_to_usart((uint8_t *) &crc, 4);
		dbg("\n\r");
	}

	stored_crc = get_stored_infoblock_crc();
	dbg("Stored CRC:\t");
	write_message_to_usart((uint8_t *) &stored_crc, 4);
	dbg("\n\r");

	if (stored_crc != crc)
	{
		dbg("Warning: info block CRC value doesn't match the calculated!\n\r");
	}
	else
	{
		dbg("Info block CRC value OK.\n\n\r");
	}

	/* Test the report id to object type / instance mapping: get the maximum
	 * report id and print the report id map. */

	dbg("Report ID to Object type / instance mapping:\n\r");

	for (report_id = 0; report_id <= max_report_id; report_id++)
	{
		object_type = report_id_to_type(report_id, &instance);
		dbg("[TSP] Report ID : %d, Object Type : T%d, Instance : %d\n\r",report_id ,object_type,instance);
	}

#ifdef CHIP_NOINIT
	object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9,0);

	status = read_U16(object_address+18, &xres);
	status = read_U16(object_address+20, &yres);
	status = read_mem(object_address+3, 2, (U8 *) xysize);
	dbg("[TSP] Read Chip xyInfo : xres(%d), yres(%d), xsize(%d), ysize(%d)\n\r",xres , yres, xysize[0], xysize[1]);

	if(Chip_NoInit == false){
		if(!((yres == (SCREEN_RESOLUTION_X-1) || yres == SCREEN_RESOLUTION_X) && xysize[0] == T9_XSIZE && xysize[1] == T9_YSIZE))
			Chip_NoInit = true;
	}    
#endif

#ifdef OPTION_WRITE_CONFIG
	qt_Power_Config_Init();
	qt_Acquisition_Config_Init();
	qt_Multitouchscreen_Init();
	qt_KeyArray_Init();
	qt_ComcConfig_Init();
	qt_Gpio_Pwm_Init();
	qt_Grip_Face_Suppression_Config_Init();
	qt_Noise_Suppression_Config_Init();
	qt_Proximity_Config_Init();
	qt_One_Touch_Gesture_Config_Init();
	qt_Selftest_Init();
#ifndef __VER_2_0__
	qt_Two_touch_Gesture_Config_Init();
#endif

	qt_CTE_Config_Init();

	if(IS_QT602240_MXT224E(tsp_family_id)){
		dbg("===============================================\n\r");
		dbg("THIS IS QT602240_MXT224E VERSION...............\n\r");
		dbg("===============================================\n\r");
		
		qt_Grip_Suppression_T40_Config_Init();
		qt_Proci_palmsuppression_T41_config_Init();
		qt_Touch_Suppression_T42_Config_Init();
		qt_CTE_T46_Config_Init();
		qt_Stylus_T47_Config_Init();
		qt_Noisesuppression_T48_config_Init();
	}

	if(IS_QT602240_MXT768E(tsp_family_id)){
		dbg("===============================================\n\r");
		dbg("THIS IS QT602240_MXT768E VERSION...............\n\r");
		dbg("===============================================\n\r");

		qt_Grip_Suppression_T40_Config_Init();
		qt_Proci_palmsuppression_T41_config_Init();
		qt_Touch_Suppression_T42_Config_Init();
		qt_CTE_T46_Config_Init();
		qt_Stylus_T47_Config_Init();
		qt_Noisesuppression_T48_config_Init();

		qt_Digitizer_T43_Config_Init();
		qt_Proximitykey_T52_Config_Init();
		qt_Gendatasource_T53_Config_Init();
	}

	/* Backup settings to NVM. */
	if (backup_config() != WRITE_MEM_OK)
	{
		dbg("Failed to backup, exiting...\n\r");
		return;
	}
	else
	{
		dbg("Backed up the config to non-volatile memory!\n\r");
	}

#else // OPTION_WRITE_CONFIG
	dbg("Chip setup sequence was bypassed!\n\r");
#endif /* OPTION_WRITE_CONFIG */

	/* Calibrate the touch IC. */
	if (calibrate_chip() != WRITE_MEM_OK)
	{
		dbg("Failed to calibrate, exiting...\n\r");
		return;
	}
	else
	{
		dbg("Chip calibrated!\n\r");
	}

	touch_data_init();

#ifdef CHARGER_MODE
	pre_charger_mode = -1;
#endif  

	dbg_func_out();
	dbg("\nWaiting for touch chip messages...\n\n\r");
}

/*------------------------------ Sub functions -----------------------------------*/
/*!
  \brief Initializes touch driver.

  This function initializes the touch driver: tries to connect to given 
  address, sets the message handler pointer, reads the info block and object
  table, sets the message processor address etc.

  @param I2C_address is the address where to connect.
  @param (*handler) is a pointer to message handler function.
  @return DRIVER_SETUP_OK if init successful, DRIVER_SETUP_FAILED 
  otherwise.
  */
uint8_t init_touch_driver(uint8_t I2C_address)
{
	int i;
	int current_report_id = 0;
	uint8_t tmp;
	uint16_t current_address;
	uint16_t crc_address;
	object_t *object_table;
	info_id_t *id;
	uint32_t *CRC;
	uint8_t status;

	dbg_func_in();

#ifdef QT_FIRMUP_ENABLE
	QT_i2c_address = I2C_address;
#endif

	/* Try to connect. */
	if(init_I2C(I2C_address) != CONNECT_OK)
	{
		dbg("[TSP][ERROR] 1\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Read the info block data. */
	id = (info_id_t *) kmalloc(sizeof(info_id_t), GFP_KERNEL | GFP_ATOMIC);
	if (id == NULL)
	{
		return(DRIVER_SETUP_INCOMPLETE);
	}

	if (read_id_block(id) != 1)
	{
		dbg("[TSP][ERROR] can't read info block data.\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}  

	/* Read object table. */
	object_table = (object_t *) kmalloc(id->num_declared_objects * sizeof(object_t), GFP_KERNEL | GFP_ATOMIC);
	if (object_table == NULL)
	{
		dbg("[TSP][ERROR] 3\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Reading the whole object table block to memory directly doesn't work cause sizeof object_t isn't necessarily the same on every compiler/platform due to alignment issues. Endianness can also cause trouble. */
	current_address = OBJECT_TABLE_START_ADDRESS;

	for (i = 0; i < id->num_declared_objects; i++)
	{
		status = read_mem(current_address, 1, &(object_table[i]).object_type);
		if (status != READ_MEM_OK)
		{
			dbg("[TSP][ERROR] 4\n");
			return(DRIVER_SETUP_INCOMPLETE);
		}
		current_address++;

		status = read_U16(current_address, &object_table[i].i2c_address);
		if (status != READ_MEM_OK)
		{
			dbg("[TSP][ERROR] 5\n");
			return(DRIVER_SETUP_INCOMPLETE);
		}
		current_address += 2;

		status = read_mem(current_address, 1, (U8*)&object_table[i].size);
		if (status != READ_MEM_OK)
		{
			dbg("[TSP][ERROR] 6\n");
			return(DRIVER_SETUP_INCOMPLETE);
		}
		current_address++;

		status = read_mem(current_address, 1, &object_table[i].instances);
		if (status != READ_MEM_OK)
		{
			dbg("[TSP][ERROR] 7\n");
			return(DRIVER_SETUP_INCOMPLETE);
		}
		current_address++;

		status = read_mem(current_address, 1, &object_table[i].num_report_ids);
		if (status != READ_MEM_OK)
		{
			dbg("[TSP][ERROR] 8\n");
			return(DRIVER_SETUP_INCOMPLETE);
		}
		current_address++;

		max_report_id += (object_table[i].num_report_ids * (object_table[i].instances+1));

		/* Find out the maximum message length. */
		if (object_table[i].object_type == GEN_MESSAGEPROCESSOR_T5)
		{
			max_message_length = object_table[i].size + 1;
		}
	}

	/* Check that message processor was found. */
	if (max_message_length == 0)
	{
		dbg("[TSP][ERROR] 9\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Read CRC. */
	CRC = (uint32_t *) kmalloc(sizeof(info_id_t), GFP_KERNEL | GFP_ATOMIC);
	if (CRC == NULL)
	{
		dbg("[TSP][ERROR] 10\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	info_block = kmalloc(sizeof(info_block_t), GFP_KERNEL | GFP_ATOMIC);
	if (info_block == NULL)
	{
		dbg("err\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	info_block->info_id = *id;
	info_block->objects = object_table;
	crc_address = OBJECT_TABLE_START_ADDRESS + id->num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;

	status = read_mem(crc_address, 1u, &tmp);
	if (status != READ_MEM_OK)
	{
		dbg("[TSP][ERROR] 11\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}
	info_block->CRC = tmp;

	status = read_mem(crc_address + 1u, 1u, &tmp);
	if (status != READ_MEM_OK)
	{
		dbg("[TSP][ERROR] 12\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}
	info_block->CRC |= (tmp << 8u);

	status = read_mem(crc_address + 2, 1, &info_block->CRC_hi);
	if (status != READ_MEM_OK)
	{
		dbg("[TSP][ERROR] 13\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Store message processor address, it is needed often on message reads. */
	message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);
	if (message_processor_address == 0)
	{
		dbg("[TSP][ERROR] 14 !!\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Store command processor address. */
	command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
	if (command_processor_address == 0)
	{
		dbg("[TSP][ERROR] 15\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}


	dbg("[TSP][max_message_length] %d\n", max_message_length);


	quantum_msg = kmalloc(max_message_length, GFP_KERNEL | GFP_ATOMIC);
	if (quantum_msg == NULL)
	{
		dbg("[TSP][ERROR] 16\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Allocate memory for report id map now that the number of report id's 
	 * is known. */

	dbg("[TSP][max_report_id] %d\n", max_report_id);	

	report_id_map = kmalloc(sizeof(report_id_map_t) * max_report_id, GFP_KERNEL | GFP_ATOMIC);

	if (report_id_map == NULL)
	{
		dbg("[TSP][ERROR] 17\n");
		return(DRIVER_SETUP_INCOMPLETE);
	}

	/* Report ID 0 is reserved, so start from 1. */

	report_id_map[0].instance = 0;
	report_id_map[0].object_type = 0;
	current_report_id = 1;

	for (i = 0; i < id->num_declared_objects; i++)
	{
		if (object_table[i].num_report_ids != 0)
		{
			int instance;
			for (instance = 0; instance <= object_table[i].instances; instance++)
			{
				int start_report_id = current_report_id;
				for (; current_report_id < (start_report_id + object_table[i].num_report_ids); current_report_id++)
				{
					dbg("[TSP][%d][current_report_id] %d, instances: %d\n", 
						i, current_report_id, instance);
					report_id_map[current_report_id].instance = instance;
					report_id_map[current_report_id].object_type = object_table[i].object_type;
				}
			}
		}
	}
	driver_setup = DRIVER_SETUP_OK;

	/* Initialize the pin connected to touch ic pin CHANGELINE to catch the
	 * falling edge of signal on that line. */

	dbg_func_out();
	return(DRIVER_SETUP_OK);
}

void read_all_register(void)
{
	U16 addr = 0;
	U8 msg;
	U16 calc_addr = 0;

	dbg_func_in();

	for(addr = 0 ; addr < 1273 ; addr++)
	{
		calc_addr = addr;

		if(read_mem(addr, 1, &msg) == READ_MEM_OK)
		{
			dbg("(0x%2x)", msg);
			if( (addr+1) % 10 == 0)
			{
				dbg("\n");
				dbg("%2d : ", addr+1);
			}

		}else
		{
			dbg("\n\n[TSP][ERROR] %s() read fail !! \n", __FUNCTION__);
		}
	}

	dbg_func_out();
}

void  clear_event(uint8_t clear)
{
	uint8_t valid_area;
	int i;   

	dbg_func_in();

	valid_area = 0;
	for ( i= 0; i<MAX_NUM_FINGER; i++ )
	{
		if(fingerInfo[i].mode == TSC_EVENT_WINDOW)
		{
			dbg("[TSP] Finger[%d] Up(TSC_EVENT_WINDOW) XY(%d, %d) - forced event\n", i, fingerInfo[i].realx, fingerInfo[i].realy);                     
			if(i == 0 && clear != TSC_CLEAR_ALL)                
			{
				dbg("TSP_Fin=%d,Down_WIN_XY(%d, %d)-forced\n", i, fingerInfo[i].realx, fingerInfo[i].realy);
				input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
				input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, fingerInfo[i].realx);
				input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].realy);
				input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, 1);//pressure[i]); // 0???? Release, ?????? Press ????(Down or Move)
				input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);            // Size ???? ID ????
				input_mt_sync(qt602240_data->input_dev);
				input_sync(qt602240_data->input_dev);
			}
			else
			{
				dbg("TSP_Fin=%d,Up_WIN_XY(%d, %d)-forced\n", i, fingerInfo[i].realx, fingerInfo[i].realy);
				input_report_key(qt602240_data->input_dev, BTN_TOUCH, 0);
				input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, fingerInfo[i].realx);
				input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].realy);
				input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, 0);//pressure[i]); // 0???? Release, ?????? Press ????(Down or Move)
				input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);            // Size ???? ID ????
				input_mt_sync(qt602240_data->input_dev);
				input_sync(qt602240_data->input_dev);
			}   
			valid_area |= 1;
		}
	}
	if(valid_area != 0)
		input_sync(qt602240_data->input_dev);

	for ( i= 0; i<MAX_NUM_FINGER; i++ )
	{
		if((fingerInfo[i].mode == TSC_EVENT_MENU) || (fingerInfo[i].mode == TSC_EVENT_HOME) || (fingerInfo[i].mode == TSC_EVENT_BACK))
		{
			switch (fingerInfo[i].mode) 
			{
				case TSC_EVENT_MENU:
					dbg("[TSP] Finger[%d] (TSC_EVENT_MENU = 0) - forced event\n", i);
					input_report_key(qt602240_data->input_dev, KEY_MENU, 0);
					break;
				case TSC_EVENT_HOME:		
					dbg("[TSP] Finger[%d] (TSC_EVENT_HOME = 0) - forced event\n", i);
					input_report_key(qt602240_data->input_dev, KEY_HOME, 0);
					break;		
				case TSC_EVENT_BACK:
					dbg("[TSP] Finger[%d] (TSC_EVENT_BACK = 0) - forced event\n", i);
					input_report_key(qt602240_data->input_dev, KEY_BACK, 0);
					break;	
				default:
					break;
			}
		}
	}

	if(clear == TSC_CLEAR_ALL)
	{
		for ( i= 0; i<MAX_NUM_FINGER; i++ )
		{
			fingerInfo[i].mode = TSC_EVENT_NONE;
			fingerInfo[i].status = -1;
			fingerInfo[i].area = 0;
			keyInfo[i].update = false; 
		}     
	}

	dbg_func_out();
}

void cal_maybe_good(void)
{    
	dbg_func_in();

	/* Check if the timer is enabled */
	if(timer_enabled)
	{
		if(IS_QT602240_MXT768E(tsp_family_id))
			cal_check_flag = 0;

		if(timer_tick > 0) /* Check if the timer timedout of 0.5seconds */
		{
			/* cal was good - don't need to check any more */
			cal_check_flag = 0;
			/* Disable the timer */
			del_timer(&timer);
			timer_enabled = false;      
			timer_tick = 0; 

			is_cal_success = true;

			if(facesup_flag){
				clear_event(TSC_CLEAR_ALL);
				facesup_flag = 0; 
			}    

			/* Write back the normal acquisition config to chip. */
			if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
			{
				/* "Acquisition config write failed!\n" */
				dbg("[TSP] [ERROR] line : %d\n", __LINE__);
				//ret = WRITE_MEM_FAILED; /* calling function should retry calibration call */
			}
			//dbg("[TSP] calibration really good\n");
		}
		else
		{
			if(!IS_QT602240_MXT768E(tsp_family_id))
				cal_check_flag = 1u;
		}
	}
	else
	{
		/* Timer not enabled, so enable it */
		timer.expires = jiffies + msecs_to_jiffies(500);
		add_timer(&timer);
		timer_enabled = true;
		timer_tick = 0u;
		cal_check_flag = 1u;
	}   

	dbg_func_out();

}


/* Calibration Checking routine - called from interrupt handler */

/*!
 * \brief Used to ensure that calibration was good
 *
 * This function will check if the last calibration was good.
 * 
 * It should be called on every touch message whilst 'cal_check_flag' is set,
 * it will recalibrate the chip if the calibration is bad. If the calibration
 * is good it will restore the ATCHCALST and ATCHCALSTHR settings in the chip 
 * so that we do not get water issues.
 *
 *
 */
int8_t check_chip_calibration(void)
{
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	uint8_t tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i;
	uint8_t j;
	uint8_t x_line_limit;
	uint8_t CAL_THR = 10;

	dbg_func_in();

	if (driver_setup != DRIVER_SETUP_OK)
		return -1;

	/* we have had the first touchscreen or face suppression message 
	 * after a calibration - check the sensor state and try to confirm if
	 * cal was good or bad */

	/* get touch flags from the chip using the diagnostic object */
	/* write command to command processor to get touch flags - 0xF3 Command required to do this */
	write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);
	/* get the address of the diagnostic object so we can get the data we need */
	diag_address = get_object_address(DEBUG_DIAGNOSTIC_T37,0);

	msleep(20); 

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	memset( data_buffer , 0xFF, sizeof( data_buffer ) );

	/* wait for diagnostic object to update */
	while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
	{
		/* wait for data to be valid  */
		if(try_ctr > 100)
		{
			/* Failed! */
			//dbg("[TSP] Diagnostic Data did not update!!\n");
			break;
		}
		msleep(5); 
		try_ctr++; /* timeout counter */
		read_mem(diag_address, 2,data_buffer);
		//dbg("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
	}

	/* data is ready - read the detection flags */
	read_mem(diag_address, 82,data_buffer);

	/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

	/* count up the channels/bits if we recived the data properly */
	if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
	{

		/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
		x_line_limit = 16 + cte_config.mode;

		if(x_line_limit > 20)
		{
			/* hard limit at 20 so we don't over-index the array */
			x_line_limit = 20;
		}

		/* double the limit as the array is in bytes not words */
		x_line_limit = x_line_limit << 1;

		/* count the channels and print the flags to the log */
		for(i = 0; i < x_line_limit; i+=2) /* check X lines - data is in words so increment 2 at a time */
		{
			/* print the flags to the log - only really needed for debugging */
			//dbg("[TSP] Detect Flags X%d, %x%x, %x%x \n", i>>1,data_buffer[3+i],data_buffer[2+i],data_buffer[43+i],data_buffer[42+i]);

			/* count how many bits set for this row */
			for(j = 0; j < 8; j++)
			{
				/* create a bit mask to check against */
				check_mask = 1 << j;

				/* check detect flags */
				if(data_buffer[2+i] & check_mask)
				{
					tch_ch++;
				}
				if(data_buffer[3+i] & check_mask)
				{
					tch_ch++;
				}

				/* check anti-detect flags */
				if(data_buffer[42+i] & check_mask)
				{
					atch_ch++;
				}
				if(data_buffer[43+i] & check_mask)
				{
					atch_ch++;
				}
			}
		}

		/* print how many channels we counted */
		//dbg("[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);

		/* send page up command so we can detect when data updates next time,
		 * page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;
		write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);

		dbg("tch_ch : %d, atch_ch : %d\n", tch_ch, atch_ch);

		/* process counters and decide if we must re-calibrate or if cal was good */      
		if((tch_ch) && (atch_ch == 0))
		{
			dbg("[TSP] calibration maybe good\n");
			cal_maybe_good();
			nCalCount =0;
			dbg_func_out();

			return 0;
		}
		else if((tch_ch + CAL_THR /*10*/ ) <= atch_ch || nCalCount > 2)
		{
			dbg("[TSP] calibration was bad\n");
			del_timer(&timer_chk_fhe);

			clear_event(TSC_CLEAR_ALL);
			touch_data_init();

			/* cal was bad - must recalibrate and check afterwards */
			calibrate_chip();

			/* Disable the timer */
			del_timer(&timer);
			timer_enabled = false;
			timer_tick = 0;
			nCalCount =0;
			dbg_func_out();

			return -1;
		}
		else
		{
			nCalCount++;
			dbg("[TSP] calibration was not decided yet\n");
			/* we cannot confirm if good or bad - we must wait for next touch 
			 * message to confirm */
			cal_check_flag = 1u;
			/* Reset the 100ms timer */
			if(!timer_enabled){
				timer.expires = jiffies + msecs_to_jiffies(500);
				add_timer(&timer);
			}   
			timer_enabled = true;
			timer_tick = 0u;

			dbg_func_out();

			return 0;
		}
	}

	dbg_func_out();

	return 0;
}

/* Just an example, __timer is used to check if timer is enabled or not
   Timer should run every 100ms */

/* Timer interrupt which overflows every 100ms */
static void check_chip_calibration_timer_func(unsigned long data)
{
	//dbg("[TSP] check_chip_calibration_timer_func\n");
	timer_tick++;
}

static void check_fhe_timer_func(unsigned long data)
{
	//dbg("[TSP] check_fhe_timer_func\n");
	timer_tick_fhe_elapse = true;
}

void  get_message(struct work_struct * p)
{
	unsigned long x, y;
	unsigned int press = 3;
	uint8_t ret_val = MESSAGE_READ_FAILED;
	int size=0,i;   
	uint8_t touch_message_flag = 0;
	uint8_t object_type, instance;
#ifdef QT_MULTITOUCH_ENABLE
	uint8_t id = 0;
	int bChangeUpDn= 0;
#endif
	uint8_t valid_area;
	uint8_t valid_key;	
	int8_t ret_checkcal;

	dbg_func_in();

	/* Get the lock */
	mutex_lock(&qt602240_data->lock);

	if (driver_setup == DRIVER_SETUP_OK)
	{
		if(timer_tick_fhe_elapse == true)
		{
			if(fherr_during_noise_timer == true)
			{
				clear_event(TSC_CLEAR_ALL);
			}    
			fherr_during_noise_timer = false;

			noise_setting_flag = false;

			timer_tick_fhe_elapse = false;
		}

		if(read_mem(message_processor_address, max_message_length, quantum_msg) == READ_MEM_OK)
		{
			/* Call the main application to handle the message. */
			dbg("[TSP] msg id =  %d, msg = %x\n", quantum_msg[0], quantum_msg[1]);

			object_type = report_id_to_type(quantum_msg[0], &instance);

			if (object_type == TOUCH_MULTITOUCHSCREEN_T9)
			{
#ifdef QT_MULTITOUCH_ENABLE
				id= quantum_msg[0] - 2;
#endif
				dbg("[TSP] T9 msg id =  %d, msg = %x\n", quantum_msg[0], quantum_msg[1]);


				/* Detect & Press */
				if( (( quantum_msg[1] & (TOUCH_DETECT|TOUCH_PRESS)) == (TOUCH_DETECT|TOUCH_PRESS)))                                
				{
					touch_message_flag = 1;
					dbg("[TSP] Down Interrupt : (msg id =  %d, msg = %x)\n", quantum_msg[0], quantum_msg[1]);
				}

				x = quantum_msg[2];
				x = x << 2;
				x = x | quantum_msg[4] >> 6;

				y = quantum_msg[3];
//				y = y << 2;
//				y = y | ((quantum_msg[4] & 0x6)  >> 2);

				y = y << 4;
				y = y | ((quantum_msg[4] & 0xf) );

				size = quantum_msg[5];

#ifdef QT_MULTITOUCH_ENABLE
				if ( (quantum_msg[1] & TOUCH_RELEASE) )    
				{
					fingerInfo[id].status= 0;
					bChangeUpDn= 1;
					dbg("\n##### Finger[%d] Up (%d,%d)!\n\n", id, fingerInfo[id].x, fingerInfo[id].y );
				}
				else if ( (quantum_msg[1] & TOUCH_DETECT) && (quantum_msg[1] & TOUCH_MOVE) )  
				{
					fingerInfo[id].id= id;
					fingerInfo[id].status= 1;
					fingerInfo[id].x= (int16_t)x;
					fingerInfo[id].y= (int16_t)y;
#if 1 //#if EF30S_BDVER_GE(EF30S_PT10) 
					bChangeUpDn= 1;
#endif

					dbg("##### Finger[%d] Move (%d,%d)!\n", id, fingerInfo[id].x, fingerInfo[id].y );
				}
				else if ( (quantum_msg[1] & TOUCH_DETECT) && (quantum_msg[1] & TOUCH_PRESS) )       
				{                               
					fingerInfo[id].id= id;
					fingerInfo[id].status= 1;
					fingerInfo[id].x= (int16_t)x;
					fingerInfo[id].y= (int16_t)y;
					bChangeUpDn= 1;
					dbg("\n##### Finger[%d] Down (%d,%d)!\n\n", id, fingerInfo[id].x, fingerInfo[id].y );
				}
#else                    
				if( ((quantum_msg[1] & TOUCH_DETECT) == TOUCH_DETECT ) && ((quantum_msg[1] & TOUCH_PRESS) == TOUCH_PRESS) )   
				{
					press = 1;
					btn_report = 1;

					//input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
				}
				else if( ((quantum_msg[1] & TOUCH_DETECT) == TOUCH_DETECT ) && ((quantum_msg[1] & TOUCH_MOVE) == TOUCH_MOVE) )    
				{
					press = 1;
				}       

				else if( ((quantum_msg[1] & TOUCH_RELEASE ) == TOUCH_RELEASE))   
				{
					press = 0;

					input_report_key(qt602240_data->input_dev, BTN_TOUCH, 0);
				}
#endif                  
				else
				{
					press = 3;
					dbg("[TSP] Unknown state ! status = %d \n",quantum_msg[1]);

					goto interrupt_return;
				}

				ret_val = MESSAGE_READ_OK;
			}                     
			else if (object_type == TOUCH_KEYARRAY_T15)
			{

				dbg("[TSP] TOUCH_KEYARRAY_T15 (quantum_msg[1]: %d, quantum_msg[3]: %d, quantum_msg[4] %d)!\n\n", quantum_msg[1], quantum_msg[3], quantum_msg[4]);

			}               
			else if (object_type == GEN_COMMANDPROCESSOR_T6) 
			{
				//Reset, Overflow, SigErr, Calibration, CFG Err ...
				dbg("[TSP] msg id =  %d, status = 0x%x \n", quantum_msg[0],quantum_msg[1]);

				if((quantum_msg [1] != 0x10) && (quantum_msg [1] != 0x00))
					goto interrupt_return;
			}
			else if (object_type == PROCG_NOISESUPPRESSION_T22) 
			{
				if ((quantum_msg[1] & 0x08 ) )
				{
					dbg("[TSP] PROCG_NOISESUPPRESSION_T22, FHERR \n");

					if(noise_setting_flag == false)
					{
						clear_event(TSC_CLEAR_ALL);
						//qt_fhe_mode_config(FHE_SET);
						noise_setting_flag = true;

						del_timer(&timer_chk_fhe); //for safety
						timer_chk_fhe.expires = jiffies + msecs_to_jiffies(10000);
						add_timer(&timer_chk_fhe);
					}
					else
						fherr_during_noise_timer = true;
				}
				goto interrupt_return;
			}     
			else if (object_type == PROCI_GRIPFACESUPPRESSION_T20)
			{
				if((quantum_msg[1]&0x01) == 0x00)   
				{ 
					dbg("[TSP] Palm released\n");
					clear_event(TSC_CLEAR_ALL);
					facesup_flag = 0;
				}
				else
				{
					dbg("[TSP] Palm suppressed\n");
					clear_event(TSC_CLEAR_ALL);
					facesup_flag = 1;
				}
			}
			else if (object_type == PROCG_NOISESUPPRESSION_T48)
			{
				if(quantum_msg[4] == 0x05 || quantum_msg[4] == 0x04)
					printk("[TSP] PROCG_NOISESUPPRESSION_T48 msg id =  %d, status = 0x%x, 0x%x\n", 
						quantum_msg[0],quantum_msg[1], quantum_msg[4]);

				if(quantum_msg[4] == 0x05 || quantum_msg[4] == 0x04)
				{
					if(auto_touch_flag == AT_DISABLE_CNT)
					{
						noisesuppression_t48_config.cfg = 112; // Median filter disable
	
						if (write_noisesuppression_t48_config(noisesuppression_t48_config) != CFG_WRITE_OK)
						{
							printk("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
						}
					}
					
					auto_touch_flag = AT_STOP;
					calibrate_chip();
				}
			}	
			else if (object_type == PROCI_TOUCHSUPPRESSION_T42)
			{
				dbg("[TSP] msg id =  %d, status = 0x%x\n", quantum_msg[0],quantum_msg[1]);			

				if(quantum_msg[1] == 0x00)
				{
					printk("[TSP] palm clear event msg id =  %d, status = 0x%x\n", quantum_msg[0],quantum_msg[1]);	
					clear_event(TSC_CLEAR_ALL);
				}
			}
			else    
			{
				dbg("[TSP] msg id =  %d, status = 0x%x \n", quantum_msg[0],quantum_msg[1]);
#if 0
				dbg("[TSP] msg id =  %d ", quantum_msg[0]);

				for(i = 1; i  < max_message_length;i++)
				{
					dbg("%x ",quantum_msg[i]);

				}
				dbg("\n");
#endif
				goto interrupt_return;
			}

		}
		else {
			printk("[TSP] read_mem failed.\n");
			goto interrupt_return;
		}


		/* touch_message_flag - set when tehre is a touch message
		   facesup_message_flag - set when there is a face suppression message
		   */
		/* Check if the cal_check_flag is enabled, and if there are any touch or face suppression messages, check the calibration */      
		if(cal_check_flag == 1 && !IS_QT602240_MXT768E(tsp_family_id))
		{   
			ret_checkcal = 0;
			if(touch_message_flag || facesup_flag /*|| facesup_message_flag*/) /* Check if the chip sent a face suppression message when in calibration */
			{
				printk("[TSP] cal_check_flag :%d, touch_message_flag: %d\n", cal_check_flag,touch_message_flag);			
				ret_checkcal = check_chip_calibration();
#ifdef RECHECK_AFTER_CALIBRATION
				calibration_message = 0;
#endif

				if(facesup_flag){
					clear_event(TSC_CLEAR_ALL);
					goto interrupt_return;
				}    
			}
#ifdef RECHECK_AFTER_CALIBRATION
			else
			{
				/* The Device is calibration. */
				if(quantum_msg[0] == 0x01 && quantum_msg[1] == 0x10)
				{
					calibration_message = 1;
					ret_checkcal = -1;                  
				}

				if((quantum_msg[0] == 0x01 && quantum_msg[1] == 0x00) && calibration_message)
				{             
					calibration_message = 0;
					//                	 msleep(100); 
					ret_checkcal = check_chip_calibration();
				}
			}
#endif	

			if(ret_checkcal == -1) {
				goto interrupt_return;
			}           

		}

	}
	else {
		goto interrupt_return;
	}
	valid_area = 0;

#ifdef QT_MULTITOUCH_ENABLE
	if (bChangeUpDn)
	{   	
		for ( i= 0; i<MAX_NUM_FINGER; i++ )
		{
			//        	    	printk("X : %d, Y: %d\n", fingerInfo[i].x, fingerInfo[i].y);

			if ( fingerInfo[i].status == -1 || (fingerInfo[i].mode == TSC_EVENT_NONE && fingerInfo[i].status == 0)) 
				continue;

			if(fingerInfo[i].mode == TSC_EVENT_NONE)
			{
				//                	printk("[TSP] Finger[%d] Down(TSC_EVENT_WINDOW) XY(%d, %d)\n", i, fingerInfo[i].x, fingerInfo[i].y);
				/* Touch Window ?????? Touch ?? ???? */
				if(fingerInfo[i].y < SCREEN_RESOLUTION_Y)
				{
					dbg("[QT602240] D:(%d, %d)\n", fingerInfo[i].x, fingerInfo[i].y);

					input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
					input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);//pressure[i]); // 0???? Release, ?????? Press ????(Down or Move)				
					input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
					input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
			        input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);            // Size ???? ID ????
					input_mt_sync(qt602240_data->input_dev);
//					input_sync(qt602240_data->input_dev);
/*
					input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
					input_report_abs(qt602240_data->input_dev, ABS_X, fingerInfo[i].x);
					input_report_abs(qt602240_data->input_dev, ABS_Y, fingerInfo[i].y);
					input_sync(qt602240_data->input_dev);
*/
					fingerInfo[i].realx = fingerInfo[i].x;
					fingerInfo[i].realy = fingerInfo[i].y;
					fingerInfo[i].mode = TSC_EVENT_WINDOW;
					valid_area |= 1;

					if(auto_touch_flag == AT_START)
						auto_touch_flag = AT_ENABLE_CNT;
				}
				/* Button ?????? Touch ?? ???? */
				/* ???? ?????? ???????? ?? 2.5mm */ 
				else if(fingerInfo[i].y >= TOUCH_KEY_Y)
				{
					/* Menu button touch */
					/* ???? ?????? ???????? ?? 7mm , ?? 4mm */
					if(fingerInfo[i].x >= TOUCH_MENU_MIN && fingerInfo[i].x <= TOUCH_MENU_MAX)
					{
						fingerInfo[i].mode = TSC_EVENT_MENU;
						keyInfo[i].code = TSC_EVENT_MENU;
						keyInfo[i].status = 1;
						keyInfo[i].update = true;
					}
					/* Home button touch */
					/* ???? ?????? ???????? ?? 4mm , ?? 7mm*/
					else if(fingerInfo[i].x >= TOUCH_HOME_MIN && fingerInfo[i].x <= TOUCH_HOME_MAX)	
					{
						fingerInfo[i].mode = TSC_EVENT_HOME;          
						keyInfo[i].code = TSC_EVENT_HOME;
						keyInfo[i].status = 1;
						keyInfo[i].update = true;
					}
					/* Back button touch */
					/* ???? ?????? ???????? ?? 4mm , ?? 7mm*/
					else if(fingerInfo[i].x >= TOUCH_BACK_MIN && fingerInfo[i].x <= TOUCH_BACK_MAX)	
					{
						fingerInfo[i].mode = TSC_EVENT_BACK;          
						keyInfo[i].code = TSC_EVENT_BACK;
						keyInfo[i].status = 1;
						keyInfo[i].update = true;
					}
				}
			}
			else
			{
				if (fingerInfo[i].status == 0)
				{
					if(fingerInfo[i].mode == TSC_EVENT_WINDOW)
					{
						fingerInfo[i].realx = fingerInfo[i].x;
						if(fingerInfo[i].y >= SCREEN_RESOLUTION_Y)
							fingerInfo[i].realy = (SCREEN_RESOLUTION_Y-1);
						else
							fingerInfo[i].realy = fingerInfo[i].y;

						printk("[QT602240] U:(%d, %d)\n", fingerInfo[i].realx, fingerInfo[i].realy);

					    input_report_key(qt602240_data->input_dev, BTN_TOUCH, 0);
 						input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);
						input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, fingerInfo[i].realx);
						input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].realy);
				        input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);
						input_mt_sync(qt602240_data->input_dev);
//					    input_sync(qt602240_data->input_dev);
/*
						input_report_key(qt602240_data->input_dev, BTN_TOUCH, 0);
						input_report_abs(qt602240_data->input_dev, ABS_X, fingerInfo[i].x);
						input_report_abs(qt602240_data->input_dev, ABS_Y, fingerInfo[i].y);
						input_sync(qt602240_data->input_dev);
*/
						valid_area |= 1;
					}
					else if(fingerInfo[i].mode == TSC_EVENT_MENU){
						keyInfo[i].code = TSC_EVENT_MENU;
						keyInfo[i].status = 0;
						keyInfo[i].update = true;
					}
					else if(fingerInfo[i].mode == TSC_EVENT_HOME){
						keyInfo[i].code = TSC_EVENT_HOME;
						keyInfo[i].status = 0;
						keyInfo[i].update = true;
					}
					else if(fingerInfo[i].mode == TSC_EVENT_BACK){
						keyInfo[i].code = TSC_EVENT_BACK;
						keyInfo[i].status = 0;
						keyInfo[i].update = true;
					}
                        
					fingerInfo[i].mode = TSC_EVENT_NONE;
					//fingerInfo[i].status= -1;
				}
				else if(fingerInfo[i].mode == TSC_EVENT_WINDOW)
				{
					fingerInfo[i].realx = fingerInfo[i].x;
					if(fingerInfo[i].y >= SCREEN_RESOLUTION_Y)
						fingerInfo[i].realy = (SCREEN_RESOLUTION_Y-1);
					else
						fingerInfo[i].realy = fingerInfo[i].y;

					dbg("[TSP] Finger[%d] Move(TSC_EVENT_WINDOW) XY(%d, %d)\n", i, fingerInfo[i].realx, fingerInfo[i].realy);

					input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
					input_report_abs(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);
					input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_X, fingerInfo[i].realx);
					input_report_abs(qt602240_data->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].realy);
        			input_report_abs(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);
					input_mt_sync(qt602240_data->input_dev);
//					input_sync(qt602240_data->input_dev);
/*
					input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);
					input_report_abs(qt602240_data->input_dev, ABS_X, fingerInfo[i].realx);
					input_report_abs(qt602240_data->input_dev, ABS_Y, fingerInfo[i].realy);
					input_sync(qt602240_data->input_dev);
*/
					valid_area |= 1;
				}
				else {
					if(fingerInfo[i].status == 1 && (fingerInfo[i].y >= SCREEN_RESOLUTION_Y && fingerInfo[i].y < SCREEN_RESOLUTION_Y + 33)) {
						dbg("Move Key area to null key area\n");
						keyInfo[i].status = 0;
						keyInfo[i].update = true;
						fingerInfo[i].mode = TSC_EVENT_NONE;
					}
					else {
						continue;
					}				
				}			
			}

			if(fingerInfo[i].status == 0) fingerInfo[i].status= -1;
		}

		if(valid_area != 0)
			input_sync(qt602240_data->input_dev);
	}   

	valid_key = 0;

	for( i= 0; i<MAX_NUM_FINGER; i++)
	{
		if(keyInfo[i].update)
		{
			if(keyInfo[i].code == TSC_EVENT_MENU){
				printk("[QT602240]M:(%d)\n", keyInfo[i].status);
				input_report_key(qt602240_data->input_dev, KEY_MENU, keyInfo[i].status);
				valid_key |= 1;
			}
			else if(keyInfo[i].code == TSC_EVENT_HOME){
				printk("[QT602240]H:(%d)\n", keyInfo[i].status);
				input_report_key(qt602240_data->input_dev, KEY_HOME, keyInfo[i].status);
				valid_key |= 1;
			} 
			else if(keyInfo[i].code == TSC_EVENT_BACK){
				printk("[QT602240]B:(%d)\n", keyInfo[i].status);
				input_report_key(qt602240_data->input_dev, KEY_BACK, keyInfo[i].status);
				valid_key |= 1;
			}    
		}
		keyInfo[i].update = false;
	}

#else // QT_MULTITOUCH_ENABLE
	if(press == 1)
	{
		input_report_abs(qt602240_data->input_dev, ABS_X, x);
		input_report_abs(qt602240_data->input_dev, ABS_Y, y);
		if( btn_report == 1)
			input_report_key(qt602240_data->input_dev, BTN_TOUCH, 1);

		input_sync(qt602240_data->input_dev);
		amplitude = quantum_msg[6];
		dbg("[TSP]%s x=%3d, y=%3d, BTN=%d, size=%d, amp=%d\n",__FUNCTION__, x, y,press, size , amplitude);

	}
	else if(press == 0)
	{
		input_sync(qt602240_data->input_dev);
		amplitude = quantum_msg[6];
		dbg("[TSP]%s x=%3d, y=%3d, BTN=%d, size=%d, amp=%d\n",__FUNCTION__, x, y,press, size , amplitude);
	}
#endif // QT_MULTITOUCH_ENABLE


interrupt_return:
	mutex_unlock(&qt602240_data->lock);

	dbg_func_out();
	return ;
}

/*------------------------------ I2C Driver block -----------------------------------*/
#define I2C_M_WR 0 /* for i2c */
#define I2C_MAX_SEND_LENGTH     300

int qt602240_i2c_write(u16 reg, u8 *read_val, unsigned int len)
{
	struct i2c_msg wmsg;
	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret,i;

	dbg_func_in();

	address_pointer = reg;

	if(len+2 > I2C_MAX_SEND_LENGTH)
	{
		dbg("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}

	wmsg.addr = qt602240_data->client->addr;
	wmsg.flags = I2C_M_WR;
	wmsg.len = len + 2;
	wmsg.buf = data;

	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;

	for (i = 0; i < len; i++)
	{
		data[i+2] = *(read_val+i);
	}

	ret = i2c_transfer(qt602240_data->client->adapter, &wmsg, 1);

	dbg_func_out();

	return ret;
}

int boot_qt602240_i2c_write(u16 reg, u8 *read_val, unsigned int len)
{
	struct i2c_msg wmsg;
	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret,i;

	if(len+2 > I2C_MAX_SEND_LENGTH)
	{
		dbg("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}

	wmsg.addr = QT602240_I2C_BOOT_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = len;
	wmsg.buf = data;

	for (i = 0; i < len; i++)
	{
		data[i] = *(read_val+i);
	}

	ret = i2c_transfer(qt602240_data->client->adapter, &wmsg, 1);

	return ret;
}


int qt602240_i2c_read(u16 reg,unsigned char *rbuf, int buf_size)
{
	static unsigned char first_read=1;
	struct i2c_msg rmsg;
	int ret;
	unsigned char data[2];

	rmsg.addr = qt602240_data->client->addr;

	if(first_read == 1)
	{
		first_read = 0;
		address_pointer = reg+1;
	}

	if((address_pointer != reg) || (reg != message_processor_address))
	{
		address_pointer = reg;

		rmsg.flags = I2C_M_WR;
		rmsg.len = 2;
		rmsg.buf = data;
		data[0] = reg & 0x00ff;
		data[1] = reg >> 8;
		ret = i2c_transfer(qt602240_data->client->adapter, &rmsg, 1);
	}

	rmsg.flags = I2C_M_RD;
	rmsg.len = buf_size;
	rmsg.buf = rbuf;
	ret = i2c_transfer(qt602240_data->client->adapter, &rmsg, 1);


	return ret;
}
/* ------------------------- ????????????? -----------------*/
/*!
 * \brief Initializes the I2C interface.
 *
 * @param I2C_address_arg the touch chip I2C address.
 *
 */
U8 init_I2C(U8 I2C_address_arg)
{
	U8 rc;
	dbg_func_in();
	rc = I2C_INIT_OK;
	dbg_func_out();
	return rc;
}


/*! \brief Enables pin change interrupts on falling edge. */
void enable_changeline_int(void)
{
}

/*! \brief Returns the changeline state. */
U8 read_changeline(void)
{
	return 0;
}

/*! \brief Maxtouch Memory read by I2C bus */
U8 read_mem(U16 start, U8 size, U8 *mem)
{
	int ret;
	U8 rc;

	memset(mem,0xFF,size);
	ret = qt602240_i2c_read(start,mem,size);
	if(ret < 0) {
		dbg("%s : i2c read failed\n",__func__);
		rc = READ_MEM_FAILED;
	}
	else
	{
		rc = READ_MEM_OK;
	}

	return rc;
}

U8 boot_read_mem(U16 start, U8 size, U8 *mem)
{
	struct i2c_msg rmsg;
	int ret;

	dbg_func_in();

	rmsg.addr = QT602240_I2C_BOOT_ADDR;
	rmsg.flags = I2C_M_RD;
	rmsg.len = size;
	rmsg.buf = mem;
	ret = i2c_transfer(qt602240_data->client->adapter, &rmsg, 1);

	dbg_func_out();

	return ret;
}

U8 read_U16(U16 start, U16 *mem)
{
	U8 status;

	status = read_mem(start, 2, (U8 *) mem);

	return status;
}

U8 write_mem(U16 start, U8 size, U8 *mem)
{
	int ret;
	U8 rc;

	ret = qt602240_i2c_write(start,mem,size);
	if(ret < 0) 
		rc = WRITE_MEM_FAILED;
	else
		rc = WRITE_MEM_OK;

	return rc;
}

U8 boot_write_mem(U16 start, U16 size, U8 *mem)
{
	int ret;
	U8 rc;

	dbg_func_in();

	ret = boot_qt602240_i2c_write(start,mem,size);
	if(ret < 0){
		dbg("boot write mem fail: %d \n",ret);
		rc = WRITE_MEM_FAILED;
	}
	else
	{
		rc = WRITE_MEM_OK;
	}

	dbg_func_out();

	return rc;
}


/*****************************************************************************
 *
 *  FUNCTION
 *  PURPOSE
 *  INPUT
 *  OUTPUT
 *
 * ***************************************************************************/

void write_message_to_usart(uint8_t msg[], uint8_t length)
{
	int i;
	dbg_func_in();

	for (i=0; i < length; i++)
	{
		dbg("0x%02x ", msg[i]);
	}
	dbg("\n\r");

	dbg_func_out();
}

/*
 * Message handler that is called by the touch chip driver when messages
 * are received.
 * 
 * This example message handler simply prints the messages as they are
 * received to USART1 port of EVK1011 board.
 */
void message_handler(U8 *msg, U8 length)
{  
}


irqreturn_t qt602240_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();
	queue_work(qt602240_wq, &qt602240_data->work);
	dbg_func_out();
	return IRQ_HANDLED;
}


static int qt602240_remove(struct i2c_client *client)
{
	dbg_func_in();

	if(qt602240_data->client->irq)
	{
		free_irq(qt602240_data->client->irq, qt602240_data);
	}
	dbg_line();
	mutex_destroy(&qt602240_data->lock);	
#ifdef SKY_PROCESS_CMD_KEY
	misc_deregister(&touch_event);
#endif
	dbg_line();
#ifdef TOUCH_IO
	misc_deregister(&touch_io);
#endif //TOUCH_IO
#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&qt602240_data->es);
#endif
	dbg_line();
	input_unregister_device(qt602240_data->input_dev);
	input_free_device(qt602240_data->input_dev);

	dbg_line();
	device_destroy(touch_atmel_class, 0);
	dbg_line();
	class_destroy(touch_atmel_class);

	dbg_line();
	if (qt602240_wq)
		destroy_workqueue(qt602240_wq);
	kfree(qt602240_data);        

	off_hw_setting();
	dbg_func_out();
	return 0;
}

#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
static int qt602240_early_suspend(struct early_suspend *h)
{
	struct qt602240_data_t *ip;

	printk("[%s] %d\n",__FUNCTION__,__LINE__);

//	return 0;

	dbg_func_in();

	ip = container_of(h, struct qt602240_data_t, es);
	disable_irq(ip->client->irq);

	del_timer(&timer);
	timer_enabled = false;
	del_timer(&timer_chk_fhe);

#ifdef CHARGER_MODE
//	qt_charger_mode_config(BATTERY_PLUGGED_AC);
#endif // CHARGER_MODE   

	qt_Power_Sleep();
	clear_event(TSC_CLEAR_ALL);

	dbg_func_out();
	return 0;
}

static int  qt602240_late_resume(struct early_suspend *h)
{
	struct qt602240_data_t *ip;

	printk("[%s] %d\n",__FUNCTION__,__LINE__);
//	return 0;

	dbg_func_in();

	ip = container_of(h, struct qt602240_data_t, es);
	touch_data_init();
	qt_Power_Config_Init();

#ifdef CHARGER_MODE
//	qt_charger_mode_config(pre_charger_mode);
#endif // CHARGER_MODE   

//	if(!IS_QT602240_MXT768E(tsp_family_id))
		calibrate_chip();

	enable_irq(ip->client->irq);

#if 0
	acquisition_config.atchcalst = 5;
	acquisition_config.atchcalsthr = 2; 

	/* Write temporary acquisition config to chip. */
	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
#endif

//	auto_touch_flag = AT_STOP;

	dbg_func_out();
	return 0;
}
#endif // CONFIG_PM && CONFIG_HAS_EARLYSUSPEND


/* I2C driver probe function */
static int __devinit qt602240_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	dbg_func_in();

	qt602240_data = kzalloc(sizeof(struct qt602240_data_t), GFP_KERNEL);
	if (qt602240_data == NULL)
	{
		pr_err("qt602240_data is not NULL.\n");
		return -ENOMEM;
	}
	
	qt602240_data->client = client;

	qt602240_wq = create_singlethread_workqueue("qt602240_wq");
	if (!qt602240_wq)
	{
		pr_err("create_singlethread_workqueue(qt602240_wq) error.\n");
		return -ENOMEM;
	}
	
	if(!touch_atmel_class)
		touch_atmel_class=class_create(THIS_MODULE, "touch_ateml");

	ts_dev = device_create(touch_atmel_class, NULL, 0, NULL, "ts");
	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(ts_dev, &dev_attr_i2c) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_i2c.attr.name);
	if (device_create_file(ts_dev, &dev_attr_setup) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_setup.attr.name);

	dbg("ts_dev creation : success.\n");

	dbg("+-----------------------------------------+\n");
	dbg("|  Quantum Touch Driver Probe!            |\n");
	dbg("+-----------------------------------------+\n");

	INIT_WORK(&qt602240_data->work, get_message );


	qt602240_data->input_dev = input_allocate_device();
	if (qt602240_data->input_dev == NULL)
	{
		rc = -ENOMEM;
		pr_err("qt602240_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	qt602240_data->input_dev->name = "qt602240_ts_input";
	set_bit(EV_SYN, qt602240_data->input_dev->evbit);
	set_bit(EV_KEY, qt602240_data->input_dev->evbit);
	set_bit(EV_ABS, qt602240_data->input_dev->evbit);
	set_bit(BTN_TOUCH, qt602240_data->input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, qt602240_data->input_dev->propbit);

	set_bit(KEY_MENU, qt602240_data->input_dev->keybit);
	set_bit(KEY_HOME, qt602240_data->input_dev->keybit);
	set_bit(KEY_BACK, qt602240_data->input_dev->keybit);

	set_bit(EV_ABS, qt602240_data->input_dev->evbit);

#ifdef SKY_PROCESS_CMD_KEY
	set_bit(KEY_SEARCH, qt602240_data->input_dev->keybit);
	set_bit(KEY_HOME, qt602240_data->input_dev->keybit);

	set_bit(KEY_0, qt602240_data->input_dev->keybit);
	set_bit(KEY_1, qt602240_data->input_dev->keybit);
	set_bit(KEY_2, qt602240_data->input_dev->keybit);
	set_bit(KEY_3, qt602240_data->input_dev->keybit);
	set_bit(KEY_4, qt602240_data->input_dev->keybit);
	set_bit(KEY_5, qt602240_data->input_dev->keybit);
	set_bit(KEY_6, qt602240_data->input_dev->keybit);
	set_bit(KEY_7, qt602240_data->input_dev->keybit);
	set_bit(KEY_8, qt602240_data->input_dev->keybit);
	set_bit(KEY_9, qt602240_data->input_dev->keybit);
	set_bit(0xe3, qt602240_data->input_dev->keybit); /* '*' */
	set_bit(0xe4, qt602240_data->input_dev->keybit); /* '#' */

	set_bit(KEY_LEFTSHIFT, qt602240_data->input_dev->keybit);
	set_bit(KEY_RIGHTSHIFT, qt602240_data->input_dev->keybit);


	set_bit(KEY_LEFT, qt602240_data->input_dev->keybit);
	set_bit(KEY_RIGHT, qt602240_data->input_dev->keybit);
	set_bit(KEY_UP, qt602240_data->input_dev->keybit);
	set_bit(KEY_DOWN, qt602240_data->input_dev->keybit);
	set_bit(KEY_ENTER, qt602240_data->input_dev->keybit);

	set_bit(KEY_SEND, qt602240_data->input_dev->keybit);
	set_bit(KEY_END, qt602240_data->input_dev->keybit);

	set_bit(KEY_VOLUMEUP, qt602240_data->input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, qt602240_data->input_dev->keybit);

	set_bit(KEY_CLEAR, qt602240_data->input_dev->keybit);

	set_bit(KEY_CAMERA, qt602240_data->input_dev->keybit);
	//    set_bit(KEY_HOLD, qt602240_data->input_dev->keybit);
#endif // SKY_PROCESS_CMD_KEY

	input_set_abs_params(qt602240_data->input_dev, ABS_X, 0, SCREEN_RESOLUTION_X, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_Y, 0, SCREEN_RESOLUTION_Y, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
#ifdef QT_MULTITOUCH_ENABLE
	input_set_abs_params(qt602240_data->input_dev, ABS_MT_POSITION_X, 0, SCREEN_RESOLUTION_X-1, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_RESOLUTION_Y-1, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(qt602240_data->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#endif  

	rc = input_register_device(qt602240_data->input_dev);
	if (rc) {
		pr_err("qt602240_probe: Unable to register %s input device\n", qt602240_data->input_dev->name);
		goto err_input_register_device_failed;
	}
	dbg("input_register_device : success.\n");

#ifdef TOUCH_IO  
	rc = misc_register(&touch_io);
	if (rc) 
	{
		pr_err("::::::::: can''t register qt602240 misc\n");
	}
#endif //TOUCH_IO

	mutex_init(&qt602240_data->lock);

#ifdef QT_FIRMUP_ENABLE
	QT_reprogram();
#else
	quantum_touch_probe();
#endif

#ifdef CHIP_NOINIT
	if(Chip_NoInit)
	{
		TSP_Restart();
		quantum_touch_probe();

		Chip_NoInit = false;
	}
#endif

	qt602240_data->client->irq = IRQ_TOUCH_INT;
	rc = request_irq(qt602240_data->client->irq, qt602240_irq_handler, IRQF_TRIGGER_FALLING, "qt602240-irq", qt602240_data);
	if (!rc)
	{
		dbg("request_irq : success.\n");
		dbg("qt602240_probe: Start touchscreen %s\n", qt602240_data->input_dev->name);
	}
	else
	{
		printk("[QT602240]request_irq failed : %d\n", rc);
	}

#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
	qt602240_data->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	qt602240_data->es.suspend = (void*)qt602240_early_suspend;
	qt602240_data->es.resume = (void*)qt602240_late_resume;
	register_early_suspend(&qt602240_data->es);
#endif

#ifdef SKY_PROCESS_CMD_KEY
	rc = misc_register(&touch_event);
	if (rc) {
		pr_err("::::::::: can''t register touch_fops\n");
	}
#endif    

	init_timer(&timer);
	timer.function = check_chip_calibration_timer_func;
	timer.data = (unsigned long)(&timer_tick);        

	init_timer(&timer_chk_fhe);
	timer_chk_fhe.function = check_fhe_timer_func;
	timer_chk_fhe.data = (unsigned long)(&timer_tick_fhe_elapse);

#ifdef PANTECH_GPIO_KEY
	rc = request_irq (gpio_to_irq(GPIO_VOL_UP), pantech_gpio_vol_up_key, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,	"keypad-volup",qt602240_data);

	rc = request_irq (gpio_to_irq(GPIO_VOL_DOWN), pantech_gpio_vol_down_key, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,	"keypad-voldown",qt602240_data);
#endif

	TSP_Restart();

	dbg_func_out();
	return 0;

err_input_register_device_failed:
	input_free_device(qt602240_data->input_dev);

err_input_dev_alloc_failed:
	kfree(qt602240_data);
	pr_err("qt602240 probe failed: rc=%d\n", rc);

	return rc;
}

static int gpio_chg  =0;

#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_BASE			(PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_MPP_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_MPP_BASE)
#define PM8921_IRQ_BASE			(NR_MSM_IRQS + NR_GPIO_IRQS)

static int init_hw_setting(void)
{
	struct regulator *vreg_touch_3_3, *vreg_touch_1_8;
	struct regulator *vreg_touch_1_8_50L;
	int rc =0;

	vreg_touch_1_8_50L = regulator_get(NULL, "8921_lvs5");
	
	rc = regulator_enable(vreg_touch_1_8_50L);
	
	vreg_touch_1_8 = regulator_get(NULL, "8921_l21");
	
	rc = regulator_set_voltage(vreg_touch_1_8, 1800000, 1800000);
	
	if (rc) { 
		pr_err("set_voltage 8921_l21 failed, rc=%d\n", rc);
		return -EINVAL;
	}

	vreg_touch_3_3 = regulator_get(NULL, "8921_l17");
	
	rc = regulator_set_voltage(vreg_touch_3_3, 2900000, 2900000);
	
	if (rc) { 
		pr_err("set_voltage 8921_l17 failed, rc=%d\n", rc);
		return -EINVAL;
	}
	
	rc = regulator_enable(vreg_touch_1_8);
	if (rc) { 
		pr_err("regulator_enable vreg_touch_1_8 failed, rc=%d\n", rc);
		return -EINVAL;
	}
	
	rc = regulator_enable(vreg_touch_3_3);
	if (rc) { 
		pr_err("regulator_enable vreg_touch_3_3 failed, rc=%d\n", rc);
		return -EINVAL;
	}
	
#if CONFIG_BOARD_VER == CONFIG_PT10 && !defined(CONFIG_MACH_APQ8064_EF50L)
	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(27), "touch_ldo1");
	if (rc) {
		pr_err("gpio_request GPIO_TOUCH_LDO1 failed, rc=%d\n",rc);
		return -EINVAL;
	}
	rc = gpio_direction_output(PM8921_GPIO_PM_TO_SYS(27), 1);
	if (rc) {
		pr_err("gpio_direction_output GPIO_TOUCH_LDO1 failed, rc=%d\n", rc);
		return -EINVAL;
	}
	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(32), "touch_ldo2");
	if (rc) {
		pr_err("gpio_request GPIO_TOUCH_LDO2 failed, rc=%d\n", rc);
		return -EINVAL;
	}
	rc = gpio_direction_output(PM8921_GPIO_PM_TO_SYS(32), 1);
	if (rc) {
		pr_err("gpio_direction_output GPIO_TOUCH_LDO2 failed, rc=%d\n",rc);
		return -EINVAL;
	}
#endif

	rc = gpio_request(GPIO_TOUCH_RST, "touch_rst");
	if (rc) {
		pr_err("gpio_request GPIO_TOUCH_RST : %d failed, rc=%d\n",GPIO_TOUCH_RST, rc);
		return -EINVAL;
	}
	
	rc = gpio_direction_output(GPIO_TOUCH_RST, 1);
	if (rc) {
		pr_err("gpio_direction_output GPIO_TOUCH_RST : %d failed, rc=%d\n",GPIO_TOUCH_RST, rc);
		return -EINVAL;
	}
  
	rc = gpio_request(GPIO_TOUCH_CHG, "touch_chg");
	if (rc) {
		pr_err("gpio_request GPIO_TOUCH_CHG : %d failed, rc=%d\n",GPIO_TOUCH_CHG, rc);
		return -EINVAL;
	}  
  
	rc = gpio_direction_input(GPIO_TOUCH_CHG);
	if (rc) {
		pr_err("gpio_direction_input gpio_chg : %d failed, rc=%d\n",gpio_chg, rc);
		return -EINVAL;
	}
  
	return 0;
}

void off_hw_setting(void)
{
	msleep(100);
}

void  qt602240_front_test_init(void)
{
	init_hw_setting();
	quantum_touch_probe();
	return ;
}

static ssize_t i2c_show(struct device *dev, struct device_attribute *attr, char *buf)
{       
	int ret;
	unsigned char read_buf[5];

	dbg_func_in();

	ret = qt602240_i2c_read(0,read_buf, 5);
	if (ret < 0) {
		dbg("qt602240 i2c read failed.\n");
	}

	dbg_func_out();
	return sprintf(buf, "%s\n", buf);
}

static ssize_t i2c_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	dbg_func_in();
	dbg_func_out();
	return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	dbg_func_in();
	dbg_func_out();
	return sprintf(buf, "%s\n", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	dbg_func_in();

	/*
	   if(strncmp(buf, "ENHIGH", 6) == 0 || strncmp(buf, "enhigh", 6) == 0) {
	   gpio_set_value(GPIO_TOUCH_EN, 1);
	   dbg("set TOUCH_EN High.\n");
	   msleep(100);
	   }
	   if(strncmp(buf, "ENLOW", 5) == 0 || strncmp(buf, "enlow", 5) == 0) {
	   gpio_set_value(GPIO_TOUCH_EN, 0);
	   dbg("set TOUCH_EN Low.\n");
	   msleep(100);
	   }
	   */
	if(strncmp(buf, "RSTHIGH", 7) == 0 || strncmp(buf, "rsthigh", 7) == 0) {
		gpio_set_value(GPIO_TOUCH_RST, 1);
		dbg("(touch)set TOUCH_RST High.\n");
		msleep(100);
	}
	if(strncmp(buf, "RSTLOW", 6) == 0 || strncmp(buf, "rstlow", 6) == 0) {
		gpio_set_value(GPIO_TOUCH_RST, 0);
		dbg("(touch) TOUCH_RST Low.\n");
		msleep(100);
	}

	dbg_func_out();
	return size;
}

void TSP_Restart(void)
{
	dbg_func_in();

	msleep(10);
	gpio_set_value(GPIO_TOUCH_RST, 0);
	dbg("(skytouch)set TOUCH_RST High.\n");
	msleep(100);

	gpio_set_value(GPIO_TOUCH_RST, 1);
	dbg("(skytouch)set TOUCH_RST Low.\n");
	msleep(10);
}

#ifdef QT_FIRMUP_ENABLE
uint8_t boot_unlock(void)
{
	int ret;
	unsigned char data[2];
	uint8_t rc;

	dbg_func_in();

	//   read_buf = (char *)kmalloc(size, GFP_KERNEL | GFP_ATOMIC);
	data[0] = 0xDC;
	data[1] = 0xAA;

	ret = boot_qt602240_i2c_write(0,data,2);
	if(ret < 0) {
		dbg("%s : i2c write failed\n",__func__);
		rc = WRITE_MEM_FAILED;
	}
	else
	{
		rc = WRITE_MEM_OK;
	}

	dbg_func_out();

	return rc;
}

uint8_t QT_Boot(bool withReset)
{
	unsigned char	boot_status;
	unsigned char	retry_cnt, retry_cnt_max;
	unsigned long int	character_position = 0;
	unsigned int	frame_size = 0;
	unsigned int	next_frame = 0;
	unsigned int	crc_error_count = 0;
	unsigned int	size1,size2;
	uint8_t			data = 0xA5;
	uint8_t			reset_result = 0;
	unsigned char	*firmware_data;
       
	if(IS_QT602240_MXT224E(tsp_family_id)){
		firmware_data = QT602240_E_firmware;
	}
	else {
		firmware_data = QT602240_firmware;
	}

	dbg_func_in();

	if(withReset)
	{
		retry_cnt_max = 10;
		reset_result = write_mem(command_processor_address + RESET_OFFSET, 1, &data);

		if(reset_result != WRITE_MEM_OK)
		{
			for(retry_cnt =0; retry_cnt < 3; retry_cnt++)
			{
				msleep(100);
				reset_result = write_mem(command_processor_address + RESET_OFFSET, 1, &data);
				if(reset_result == WRITE_MEM_OK)
				{
					dbg("write_mem(RESET_OFFSET) : fail.\n");
					break;
				}
			}
			dbg("write_mem(RESET_OFFSET) : fail.\n");
		}
		else
		{
			dbg("write_mem(RESET_OFFSET) : fail.\n");
		}

		msleep(100);
	}
	else
	{
		retry_cnt_max = 30;
	}

	for(retry_cnt = 0; retry_cnt < retry_cnt_max; retry_cnt++)
	{
		if(boot_read_mem(0,1,&boot_status) == READ_MEM_OK)
		{
			retry_cnt = 0;
			dbg("TSP boot status is %x stage 2 \n", boot_status);

			if((boot_status & QT_WAITING_BOOTLOAD_COMMAND) == QT_WAITING_BOOTLOAD_COMMAND)
			{
				if(boot_unlock() == WRITE_MEM_OK)
				{
					msleep(10);
					dbg("Unlock OK\n");
				}
				else
				{
					dbg("Unlock fail\n");
				}
			}
			else if((boot_status & 0xC0) == QT_WAITING_FRAME_DATA)
			{
				/* Add 2 to frame size, as the CRC bytes are not included */
				size1 =  *(firmware_data+character_position);
				size2 =  *(firmware_data+character_position+1)+2;
				frame_size = (size1<<8) + size2;

				dbg("Frame size:%d\n", frame_size);
				dbg("Firmware pos:%d\n", (int)character_position);
				/* Exit if frame data size is zero */
				if( 0 == (int)frame_size )
				{
					if(QT_i2c_address == I2C_BOOT_ADDR_0)
					{
						QT_i2c_address = I2C_APPL_ADDR_0;
					}
					dbg("0 == frame_size\n");
					return 1;
				}
				next_frame = 1;
				boot_write_mem(0,frame_size, (firmware_data +character_position));
				msleep(10);
				dbg(".");

			}
			else if(boot_status == QT_FRAME_CRC_CHECK)
			{
				dbg("CRC Check\n");
			}
			else if(boot_status == QT_FRAME_CRC_PASS)
			{
				if( next_frame == 1)
				{
					dbg("CRC Ok\n");
					character_position += frame_size;
					next_frame = 0;
				}
				else
				{
					dbg("next_frame != 1\n");
				}
			}
			else if(boot_status  == QT_FRAME_CRC_FAIL)
			{
				dbg("CRC Fail\n");
				crc_error_count++;
			}
			if(crc_error_count > 10)
			{
				return QT_FRAME_CRC_FAIL;
			}
		}
	}
	dbg_func_out();
	return (0);
}

/* qt602240 chipset version check */
void QT_reprogram(void)
{
	uint8_t version=0, build=0;
	unsigned char rxdata=0;

	dbg_func_in();

	if(boot_read_mem(0,1,&rxdata) == READ_MEM_OK)
	{
		dbg("Enter to new firmware : boot mode\n");
		if(QT_Boot(0)) {
			TSP_Restart();  //cte mode == 0
			quantum_touch_probe(); //cte mode ==3 
			TSP_Restart();
		}
		quantum_touch_probe(); //cte mode ==3 
		TSP_Restart();	
		dbg("Reprogram done : boot mode\n");       
	}

	/* find and initialise QT device */
	quantum_touch_probe();

	if (driver_setup != DRIVER_SETUP_OK)
		return;

	get_version(&version);
	get_build_number(&build);

	printk("[QT602240]version=0x%08X\n", version);
	//dbg("version=0x%08X\n", version);

	// mXT224E	
	if(IS_QT602240_MXT224E(tsp_family_id))
	{
	}
	// mXT224
	else
	{
#if defined(__VER_2_0__)
	if((version < 0x20)||((version == 0x20)&&(build != 0xAA)))
#else
	if((version < 0x16)||((version == 0x16)&&(build != 0xAB)))
#endif    
	{
		dbg("Enter to new firmware : ADDR = Other Version\n");
		if(QT_Boot(1)) {
			TSP_Restart();
			quantum_touch_probe();
			TSP_Restart(); 
		}
		quantum_touch_probe();
		TSP_Restart(); 
		dbg("Reprogram done : ADDR = Other Version\n");
	}
	}

	dbg_func_out();
}
#endif

static ssize_t setup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	dbg_func_in();
	dbg_func_out();
	return 0;
}

static ssize_t setup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	dbg_func_in();
	dbg_func_out();
	return size;
}

int __init qt602240_init(void)
{
	int rc;

	dbg_func_in();
	rc = init_hw_setting();
	if(rc<0)
	{
		printk("[QT602240]qt602240 : init_hw_setting failed. (rc=%d)\n", rc);
		return rc;
	}
	dbg("i2c_add_driver\n");
	rc = i2c_add_driver(&qt602240_driver);
	dbg("rc=%d\n", rc);
	if(rc)
	{
		printk("[QT602240]qt602240 : i2c_add_driver failed. (rc=%d)\n", rc);
	}

	dbg_func_out();
	return rc;
}

void __exit qt602240_exit(void)
{
	dbg_func_in();
	
	i2c_del_driver(&qt602240_driver);
	dbg_func_out();
	return;
}

late_initcall(qt602240_init);
module_exit(qt602240_exit);

MODULE_DESCRIPTION("ATMEL qt602240 Touchscreen Driver");
MODULE_LICENSE("GPL");

