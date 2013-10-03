/*
 * Copyright (c) 2010 Pantech Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <mach/vreg.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>//jhseo test for ldo control
#include <linux/err.h>//jhseo test for ldo control
#include <linux/gpio.h>//jhseo test for ldo control

#include <linux/miscdevice.h>


#include "tchkey_pt.h"

#define TCHKEYPT_DRV_NAME	"tchkeypt"
#define DRIVER_VERSION		"1.1.0"	// for INVENSENSE

//#define MAINTOUCHIC_AVOID
//#define USE_POWER_CONTROL
#ifndef USE_POWER_CONTROL
#define WITH_POWER_MSG
#define WITH_HOLD_MSG
#define WITH_RESET_MSG
#else
#define TOUCHPAD_SCL	 			12	//54
#define TOUCHPAD_SDA	 			13	//53
#endif

#define TOUCHPAD_RST				62	//72

#define USE_TCHPAD_WORKQUEUE
#define USE_FILE_ATTR

#define SIZE_250x200				0x01
#define SIZE_200x160				0x02
#define SIZE_150x120				0x03
#define SIZE_100x100				0x04
#define SIZE_100x080				0x05
#define SIZE_060x060				0x06
#define SIZE_250x200_PART_CENTER	0x10
#define SCROL_SIZE					SIZE_100x100

#define USE_TRACKBALL				0x01
#define USE_TOUCHIC					0x02
#define USE_MOUSE					0x03
#define TOUCHPAD_MODE				USE_TOUCHIC

#define MAX_NUM_FINGER				1


#define SINGLE_TOUCH
//#define MULTI_TOUCH

//#define X_INVERSION
//#define Y_INVERSION
/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */
//#define SENSOR_TCHKEYPT_DBG_ENABLE
#ifdef SENSOR_TCHKEYPT_DBG_ENABLE
#define dbg(fmt, args...)   printk("[TCHKEYPT] " fmt, ##args)
#else
#define dbg(fmt, args...)
#endif
#define dbg_func_in()      // dbg("[FUNC_IN] %s\n", __func__)
#define dbg_func_out()     // dbg("[FUNC_OUT] %s\n", __func__)
#define dbg_line()         // dbg("[LINE] %d(%s)\n", __LINE__, __func__)
/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
/* SKY BOARD FEATURE */
/* -------------------------------------------------------------------- */
#define TCHKEYPT_PS_INT_N						72	//36
#define TCHKEYPT_PS_IRQ						gpio_to_irq(TCHKEYPT_PS_INT_N)

#define TCHKEYPT_REG_BASE						0x0000
#define TCHKEYPT_VERSION_REG					(TCHKEYPT_REG_BASE+0x0040)
#define TCHKEYPT_SEQUENCE_KEY_REG			(TCHKEYPT_REG_BASE+0x0000)

/*#define TCHKEYPT_OPERATION_MODE					0x0080
#define OPERATION_NORMAL_MODE								0x00
#define OPERATION_POWER_DOWN_MODE							0x01
*/


#define TURN_ON									0x21
#define IDLE_STATE								0x10
#define TURN_OFF								0x30
#define VD_KEY									0x42
#define VU_KEY									0x41
#define PW_KEY									0x01
#define HD_KEY									0x04


#define I2C_DONE_ADDR						0xFFFF
#define I2C_DONE_VALUE							0x01
#define IC_RESET_VALUE							0x02

//Firmware Status
#define NORMAL_MODE							0x00
#define INITIAL_PARAMETER_REQUEST			0xFF

// Event Type
#define ABSOLUTE_POINT						1
#define KEY_EVENT							2
#define GESTURE_EVENT						4
#define RELATIVE_POINT						8
#define HISTOL_MODE							64
#define CAP_MODE							128

//LCD resolution 
#define RESOLUTION_X						1080
#define RESOLUTION_Y						1920


#define WRONG_KEY_VALUE						0xff
#define ONE_FINGER_RELEASE_STATUS			0x00
#define TWO_FINGER_STATUS						0x01
#define ONE_FINGER_PRESS_STATUS				0x02
#define MAINTOUCHIC_PRESS_STATUS				0x03
#define MAINTOUCHIC_RELEASE_STATUS			0x04


#define TCHPAD_VIB_ST							2
#define TCHPAD_VIB_DR							60
#define TCHPAD_VIB_ST_PAD						6
#define TCHPAD_VIB_DR_PAD						80

#if (SCROL_SIZE==SIZE_250x200)
#define TCHPAD_MIN_X							275
#define TCHPAD_MIN_Y							540
#define TCHPAD_RANGE_X							250
#define TCHPAD_RANGE_Y							200
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									60
#define MENU_BAR								60
#define LEFT_SIDE								30
#define RIGHT_SIDE								30
#elif (SCROL_SIZE==SIZE_200x160)
#define TCHPAD_MIN_X							300
#define TCHPAD_MIN_Y							560
#define TCHPAD_RANGE_X							200
#define TCHPAD_RANGE_Y							160
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									50
#define MENU_BAR								50
#define LEFT_SIDE								20
#define RIGHT_SIDE								20
#elif (SCROL_SIZE==SIZE_150x120)
#define TCHPAD_MIN_X							325
#define TCHPAD_MIN_Y							580
#define TCHPAD_RANGE_X							150
#define TCHPAD_RANGE_Y							120
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									40
#define MENU_BAR								40
#define LEFT_SIDE								20
#define RIGHT_SIDE								20
#elif (SCROL_SIZE==SIZE_100x100)
#define TCHPAD_MIN_X							350
#define TCHPAD_MIN_Y							590
#define TCHPAD_RANGE_X							100
#define TCHPAD_RANGE_Y							100
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									30
#define MENU_BAR								30
#define LEFT_SIDE								25
#define RIGHT_SIDE								25
#elif (SCROL_SIZE==SIZE_100x080)
#define TCHPAD_MIN_X							350
#define TCHPAD_MIN_Y							600
#define TCHPAD_RANGE_X							100
#define TCHPAD_RANGE_Y							80
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									30
#define MENU_BAR								30
#define LEFT_SIDE								20
#define RIGHT_SIDE								20
#elif (SCROL_SIZE==SIZE_060x060)
#define TCHPAD_MIN_X							370
#define TCHPAD_MIN_Y							610
#define TCHPAD_RANGE_X							60
#define TCHPAD_RANGE_Y							60
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									30
#define MENU_BAR								30
#define LEFT_SIDE								20
#define RIGHT_SIDE								20
#elif (SCROL_SIZE==SIZE_250x200_PART_CENTER)
#define TCHPAD_MIN_X							275
#define TCHPAD_MIN_Y							540
#define TCHPAD_RANGE_X							250
#define TCHPAD_RANGE_Y							200
#define TCHPAD_MAX_X							(TCHPAD_MIN_X+TCHPAD_RANGE_X)
#define TCHPAD_MAX_Y							(TCHPAD_MIN_Y+TCHPAD_RANGE_Y)
#define UP_BAR									540
#define MENU_BAR								540
#define LEFT_SIDE								275
#define RIGHT_SIDE								275
#endif
#define LCDSIZE_MIN_X							(TCHPAD_MIN_X-LEFT_SIDE)
#define LCDSIZE_MIN_Y							(TCHPAD_MIN_Y-UP_BAR)
#define LCDSIZE_MAX_X							(TCHPAD_MAX_X+RIGHT_SIDE)
#define LCDSIZE_MAX_Y							(TCHPAD_MAX_Y+MENU_BAR)


#ifdef X_INVERSION
#define X_OFFSET								0
#else
#define X_OFFSET								0
#endif
#ifdef Y_INVERSION
#define Y_OFFSET								240
#else
#define Y_OFFSET								0
#endif

/* -------------------------------------------------------------------- */
/* Frimware Update */
/* -------------------------------------------------------------------- */
//static int tchkeypt_fwupdate(void);

/* -------------------------------------------------------------------- */
/* Structure */
/* -------------------------------------------------------------------- */
//static  struct tchkeyptdata_t *tchkeyptdata;

/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);

static struct i2c_driver tchkeypt_driver;
static struct tchkeyptdata_t *tchkeyptdata;


/* -------------------------------------------------------------------- */
/* Function Proto type */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void tchkey_early_suspend(struct early_suspend *handler);
static void tchkey_late_resume(struct early_suspend *handler);
#endif
static int tchkeypt_i2c_read(u16 reg, u8 *buf, int count);
//static int tchkeypt_i2c_write(u16 reg, u8 *data, int len);
static int tchkeypt_i2c_addr_write(u16 reg, u8 *buf, int count);
//static int tchkeypt_i2c_reset(struct tchkeyptdata_t *tmptchkeypt,u16 reg);

//static int tchkeypt_status_change_mode(int Mode);

static irqreturn_t tchkeypt_irq_handler(int irq, void *dev_id);
static void tchkeypt_work_f(struct work_struct *work);

#define tchkeypt_get_reg(reg,data)	tchkeypt_i2c_read(reg,data,12)
#define tchkeypt_get_id(reg,data)		tchkeypt_i2c_read(reg,data,1)

#ifdef USE_POWER_CONTROL
static void tchpad_power_control(u8 control);
#endif

#if (TOUCHPAD_MODE==USE_MOUSE)
static int temp_x_pos,temp_y_pos;
#endif

#ifdef USE_TCHPAD_WORKQUEUE
struct workqueue_struct *tchpad_wq;
#endif

#ifdef USE_FILE_ATTR
static struct class *touch_pad_class;
struct device *ts_pad_dev;
#endif


//FILE IO
static struct file_operations fops = 
{
	.owner =    THIS_MODULE,
	.read =     read,
	.write =    write,
	.open =     open,
	.release =  release
};

static struct miscdevice touch_io = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "tchkey_pt",
	.fops =     &fops
};

/* -------------------------------------------------------------------- */
/* External Functions */
/* -------------------------------------------------------------------- */
/* -------------------------------------------------------------------- */
/* Internal Functions */
/* -------------------------------------------------------------------- */
#ifdef USE_FILE_ATTR
static ssize_t setup_rear_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;

	dbg_func_in();

	enable = atomic_read(&tchkeyptdata->enable);

	dbg_func_out();

	return sprintf(buf, "%d\n", enable);
}

static ssize_t setup_rear_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	dbg_func_in();

	if(!client)
	{
		printk("[TCHKEYPT] %s: i2c_client failed \n",__func__);
		enable=0xff;
	}

	if ((enable == 0) || (enable == 1))
	{
		if(enable == 0)
		{
			/* disable worker */
			disable_irq_nosync(client->irq);

		#ifdef USE_POWER_CONTROL
			gpio_direction_output(TOUCHPAD_SCL,0);  //low
			gpio_direction_output(TOUCHPAD_SDA,0);  //low

			tchpad_power_control(0);
		#endif
		}
		else// if(enable == 1)
		{
		#ifdef USE_POWER_CONTROL
			tchpad_power_control(1);

			gpio_direction_output(TOUCHPAD_SCL,1);  //high
			gpio_direction_output(TOUCHPAD_SDA,1);  //high
		#endif
			enable_irq(client->irq);
		}

		atomic_set(&tchkeyptdata->enable, enable);
	}

	dbg_func_out();

	return size;
}


static DEVICE_ATTR(setup, S_IRUGO | S_IWUSR, setup_rear_show, setup_rear_store);
#endif

#ifdef USE_POWER_CONTROL
static struct regulator *l17_1p8v;

static void tchpad_power_control(u8 control)
{
	int rc;

	if(control==1)//power on
	{
		rc = regulator_enable(l17_1p8v);
		if (rc)
		{
			printk("[TCHKEYPT] %s: vreg enable failed (%d)\n",__func__, rc);
		}
		else
		{
			printk("[TCHKEYPT] %s OK\n", __func__);
		}
	}
	else
	{
		rc = regulator_disable(l17_1p8v);
		if (rc)
		{
			printk("[TCHKEYPT] %s: vreg enable failed (%d)\n",__func__, rc);
		}
		else
		{
			printk("[TCHKEYPT] %s OK\n", __func__);
		}
	}
}
#endif
/* ------------- I2C Interface ---------------*/
static int tchkeypt_i2c_read(u16 reg, u8 *buf, int count)
{
	int rc1, rc2;
	int ret = 0; 
	u8 cmd[1];

	dbg_func_in();

	if ( tchkeyptdata->client == NULL ) {
		printk("%s : touch power key data.client is NULL\n", __func__);
		return -1;
	}

	buf[0] = reg & 0xFF;
	buf[1] = ( reg >> 8 ) & 0xFF;
	
	rc1 = i2c_master_send(tchkeyptdata->client,  buf, 2);

	rc2 = i2c_master_recv(tchkeyptdata->client, buf, count);

	if ( (rc1 != 2) || (rc2 != count ) ) {
		printk("tchkeypt_i2c_read FAILED: read of register %x(rc1=%d/rc2=%d)\n", reg, rc1,rc2);
		ret = -1;
	}

	cmd[0] = I2C_DONE_VALUE;
	
	tchkeypt_i2c_addr_write(I2C_DONE_ADDR,&cmd[0],1);

	dbg_func_out();

	return ret;
}

//#if defined(WITH_POWER_MSG) || defined(WITH_HOLD_MSG) || defined(WITH_RESET_MSG)
/*static int tchkeypt_i2c_write(u16 reg, u8 *data, int len)
{
	u8  buf[20];
	int rc;
	int ret = 0;
	int i;
	u8 cmd[1];
	
	dbg_func_in();

	mutex_lock(&tchkeyptdata->i2clock); 

	//Added tchkeypt interrunp gpio value (low -> 1us -> high -> Input Mode)
	rc = gpio_direction_output(TCHKEYPT_PS_INT_N, 0);

	if (rc) {
		printk("gpio_direction_output TCHKEYPT_PS_INT_N 0 : %d failed, rc=%d\n",TCHKEYPT_PS_INT_N, rc);
		return -EINVAL;
	}
	
	
	udelay(1);

	
	rc = gpio_direction_output(TCHKEYPT_PS_INT_N, 1);

	if (rc) {
		printk("gpio_direction_output TCHKEYPT_PS_INT_N 1 : %d failed, rc=%d\n",TCHKEYPT_PS_INT_N, rc);
		return -EINVAL;
	}

	rc  = gpio_direction_input(TCHKEYPT_PS_INT_N);
	if (rc) {
		printk("gpio_direction_input TCHKEYPT_PS_INT_N : %d failed, rc=%d\n",TCHKEYPT_PS_INT_N, rc);
		return -EINVAL;
	}
	// end Added

	
	if ( tchkeyptdata->client == NULL ) {
		printk("%s : tchkeyptdata->client is NULL\n", __func__);
		return -ENODEV;
	}
	
	buf[0] = reg & 0xFF;
	buf[1] = ( reg >> 8 ) & 0xFF;

	if (len >= 20) {
		printk( "tchkeypt_i2c_write FAILED: buffer size is limitted(20)\n");
		return -1;
	}
	for( i=0 ; i<len; i++ ) buf[i+2] = data[i];

	rc = i2c_master_send(tchkeyptdata->client, buf, len+2);

	if (rc != len+2) {
		printk( "tchkeypt_i2c_write FAILED: writing to reg %x\n", reg);
		ret = -1;
	}


	cmd[0] = I2C_DONE_VALUE;
	
	tchkeypt_i2c_addr_write(I2C_DONE_ADDR,&cmd[0],1);

	mutex_unlock(&tchkeyptdata->i2clock); 
	dbg_func_out();

	return ret;
}
//#endif
*/
// To do I2C Done - write  value(1) at 0XFFFF
static int tchkeypt_i2c_addr_write(u16 reg, u8 *data, int count){
	u8	buf[20];
	int rc;
	int ret = 0;
	
	dbg_func_in();

	if ( tchkeyptdata->client == NULL ) {
		printk("%s : tchkeyptdata->client is NULL\n", __func__);
		return -ENODEV;
	}
	
	buf[0] = reg & 0xFF;
	buf[1] = ( reg >> 8 ) & 0xFF;

	buf[2] = data[0];
	
	rc = i2c_master_send(tchkeyptdata->client, buf, count+2);

	if (rc != 3) {
		printk( "tchkeypt_i2c_write FAILED: writing to reg %x\n", reg);
		ret = -1;
	}

	dbg_func_out();
	return ret;
}
/* -------------------------------------------------------------------- */

// State : Power down Mode & Normal Mode
/*static int tchkeypt_status_change_mode(int Mode){
	if(Mode == 1)	//Normal Mode
	{
		tchkeypt_i2c_write(struct tchkeyptdata_t * tchkeyptdata,u16 reg,u8 * data,int len)



}*/

static bool tchkey_get_available(void)
{
	u8 id=0;
	int ret = 0;
	
	dbg_func_in();

	// 1. check client
	if(tchkeyptdata->client == NULL) {
		printk("[ERR] %s : i2c client is NULL\n", __func__);
		return 0;
	}

	ret = tchkeypt_get_id(TCHKEYPT_REG_BASE, &id);
	
	if( ret ) {
		printk("[ERR] %s : tchkey id is not 0x%02X\n", __func__, TURN_OFF);
		return 0;
	}

	dbg("%s : ret = %d / id = 0x%02X / add = 0x%02X\n", __func__, ret, id,tchkeyptdata->client->addr );

	dbg_func_out();
	
	return 1;
}


/* -------------------------------------------------------------------- */
/* Frimware Update */
/* -------------------------------------------------------------------- 
static int tchkeypt_fwupdate(void)
{
	u8 cmd[1];
#if 1
#else
	u8 rom_status;
#endif
	int length,rom_addr;
	int ret = 0;
	int i = 0;

	u8 read_buffer[4096];
	u8 file_buffer[4096];
	

	// 1. Password Register Setting 
	cmd[0] = 0x6B;
	tchkeypt_i2c_addr_write(0xFFFC, cmd, 1);

	cmd[0] = 0xD2;
	tchkeypt_i2c_addr_write(0xFFFD, cmd, 1);

	// 2. Enter EEPROM Program Mode
	cmd[0] = 0x80;
	tchkeypt_i2c_addr_write(0xFFFF, cmd, 1);

	// 3. Page Register Setting
	length = 256;
	rom_addr = 0;
	for(i=0; i<length; i++,rom_addr+=128)
	{
		// a. Set to write at Page Number.
		cmd[0] = rom_addr / 128;
		if(	tchkeypt_i2c_addr_write(0xFFF9, cmd, 1))
			return -1;

		// b. Page Buffer Reset
		cmd[0] = 0x01;
		if(tchkeypt_i2c_addr_write(0xFFFE, cmd, 1))
			return -1;

		// c. Fill Buffer Load Enable(128 byte)
		cmd[0] = 0x02;
		if(tchkeypt_i2c_addr_write(0xFFFE, cmd, 1))
			return -1;

		// d. Fill Page Buffer (128 byte)
		if(tchkeypt_i2c_addr_write(0x0000, &file_buffer[rom_addr], 128))
			return -1;

		// -------------------------------------------------------------------- 
		// Erase & wait 
		// -------------------------------------------------------------------- 
		cmd[0] = 0x03;
		if(tchkeypt_i2c_addr_write(0xFFFE, cmd, 1))
			return -1;
#if 1		// Use the Delay because HW bug
		msleep(3);
#else
		while(1)
		{
			if(tchkeypt_i2c_read(0xFFFE,&rom_status,1))
				return -1;
			if(rom_status == 0)
				break;
		}
#endif

		// -------------------------------------------------------------------- 
		// Program & wait 
		// -------------------------------------------------------------------- 
		cmd[0] = 0x04;
		if(tchkeypt_i2c_addr_write(0xFFFE, cmd, 1))
			return -1;
#if 1	// Use the Delay because HW bug
		msleep(3);
#else
		while(1)
		{
			if(tchkeypt_i2c_read(0xFFFE,&rom_status,1))
				return -1;
			if(rom_status == 0)
				break;
		}
#endif
	}
	
	// -------------------------------------------------------------------- 
	// Read * Verify 
	// -------------------------------------------------------------------- 
	// Read 
	cmd[0] = 0x06;
	if(tchkeypt_i2c_addr_write(0xFFFE, cmd, 1))
		return -1;

	for(i=0, rom_addr = 0; i<length;i++,rom_addr+=128)
	{
		if(	tchkeypt_i2c_read(rom_addr,&read_buffer[rom_addr], 128))
			return -1;
	}

	for(i=0; i<length*128; i++){
		if(file_buffer[i]!=read_buffer[i]){
			printk("diff\n");
			return -1;
		}
	}

	return ret;
		
}	// end tchkeypth_fwupdate*/
	


/* ------------- Interrupt and Handler ---------------*/
static irqreturn_t tchkeypt_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();

	disable_irq_nosync(tchkeyptdata->client->irq);

#ifdef USE_TCHPAD_WORKQUEUE
	queue_work(tchpad_wq, &tchkeyptdata->work);
#else
	schedule_work(&tchkeyptdata->work);
#endif

	dbg_func_out();
	return IRQ_HANDLED;
}

static void tchkeypt_work_f(struct work_struct *work)
{
	int ret = 0;
	u8 data[12];
	int temp_x = 0;
	int temp_y = 0;
#ifdef MULTI_TOUCH
	int id = 0;
#endif
	//float d_temp_x;//,d_temp_y;
	int i = 0 ;

	dbg_func_in();
	mutex_lock(&tchkeyptdata->i2clock);  // remove for maintouch mutex

	ret |= tchkeypt_get_reg(TCHKEYPT_SEQUENCE_KEY_REG, &data[0]);

	if(ret) {
		dbg("%s : can't get tchkey value \n",__func__);
		goto err_work_exit;
	}

	for(i=0; i<12; i++){
		//dbg("data[%d] = %d\n",i,data[i]);

	}
	/************************************************************************************************************
	* data 0 : Firmware Status
	* data 1 : Event Type (Cap Mode, Histo Mode, Reserved, Reserved, Relative Point, Gesture Event, Key Event, Absolute Point )
	* data 2 : Gesture Data
	* data 3 : Valid Key
	* data 4 : Key Data(3 byte)
	* data 7 : Vaild Point ( bit enable - 1point 0x01, 2point 0x03, 3point 0x07 etc )
	* data 8 : Point X0(2 byte)
	* data10: Point Y0(2 byte)
	* data12: Point X1(2 byte)
	* data14: Point Y1(2 byte)
	* data16: Point X2(2 byte)
	* data18: Point Y2(2 byte)
	* data20: Point X3(2 byte)
	* data22: Point Y3(2 byte)
	* data24: Point X4(2 byte)
	* data26: Point Y4(2 byte)
	**************************************************************************************************************/	

	if(data[1]==ABSOLUTE_POINT && data[7]==0x01)//pad trackball gesture by one finger touch
	{
		dbg("One Finger Press Down Status\n");


		/*#ifdef X_INVERSION
			temp_x=(LCDSIZE_MAX_X+X_OFFSET)-(TCHPAD_MIN_X+data[4]);
		#else
			temp_x=TCHPAD_MIN_X+data[4];
		#endif
		#ifdef Y_INVERSION
			temp_y=(LCDSIZE_MAX_Y+Y_OFFSET)-(TCHPAD_MIN_Y+data[5]);
		#else
			temp_y=TCHPAD_MIN_Y+data[5];
		#endif*/
		//temp_x = data[9] << 8;

		
		temp_x = data[9];

		temp_x = 100 - temp_x;

		temp_y = data[11];

		dbg("Pre X position = %d\t Y Position = %d\n",temp_x,temp_y);

		temp_x = temp_x * 4;

		temp_y = temp_y * 4;


		//temp_x = (temp_x * 108) /10;
		
		//temp_y = (temp_y * 192) /10;
		//d_temp_x = temp_x;
		//d_temp_x = (d_temp_x * 108)/ 10 ;

		//temp_x = (int)(d_temp_x +0.5);


		//d_temp_y = temp_y;
		//temp_x = (int)((temp_x * RESOLUTION_X)/TCHPAD_RANGE_X +0.5);
		//temp_y = (int)((temp_y * RESOLUTION_Y)/TCHPAD_RANGE_Y +0.5);


		
		//dbg("X position = %d\t Y Position = %d\n",temp_x,temp_y);
		
		//printk("X position = %d\t Y Position = %d\n",temp_x,temp_y);

#ifdef MULTI_TOUCH
		id = input_mt_new_trkid(tchkeyptdata->tchkeypt);
		input_mt_slot(tchkeyptdata->tchkeypt, 0);
		input_report_abs(tchkeyptdata->tchkeypt, ABS_MT_TRACKING_ID, id);
		input_report_abs(tchkeyptdata->tchkeypt, ABS_MT_POSITION_X, temp_x);
		input_report_abs(tchkeyptdata->tchkeypt, ABS_MT_POSITION_Y, temp_y);
		input_report_key(tchkeyptdata->tchkeypt, BTN_TOUCH, 1);
		input_sync(tchkeyptdata->tchkeypt);
#endif

#ifdef SINGLE_TOUCH
		input_report_abs(tchkeyptdata->tchkeypt, ABS_X, temp_x);
		input_report_abs(tchkeyptdata->tchkeypt, ABS_Y, temp_y);
		input_report_key(tchkeyptdata->tchkeypt, BTN_TOUCH, 1);
		input_sync(tchkeyptdata->tchkeypt);

#endif
		
		tchkeyptdata->PAD_FUNCTION=ONE_FINGER_PRESS_STATUS;
		
		
	}
	if(data[7]==0x00 && data[8]==0xFF && data[9]==0xFF && data[10]==0xFF && data[11]==0xFF)//pad release event by no touch
	{
		//dbg("One Finger Release Status\n");
		if(tchkeyptdata->PAD_FUNCTION==ONE_FINGER_PRESS_STATUS)
		{			
#ifdef MULTI_TOUCH
		input_mt_slot(tchkeyptdata->tchkeypt, 0);
        input_report_abs(tchkeyptdata->tchkeypt, ABS_MT_TRACKING_ID, -1);
		input_report_key(tchkeyptdata->tchkeypt, BTN_TOUCH, 0);
#endif

#ifdef SINGLE_TOUCH
		input_report_abs(tchkeyptdata->tchkeypt, ABS_X, temp_x);
		input_report_abs(tchkeyptdata->tchkeypt, ABS_Y, temp_y);
		input_report_key(tchkeyptdata->tchkeypt, BTN_TOUCH, 0);

#endif

		input_sync(tchkeyptdata->tchkeypt);

		}
		tchkeyptdata->PAD_FUNCTION=ONE_FINGER_RELEASE_STATUS;
		tchkeyptdata->PAD_TEMP_FUNCTION=ONE_FINGER_RELEASE_STATUS;
	}


err_work_exit:
	enable_irq(tchkeyptdata->client->irq);
	mutex_unlock(&tchkeyptdata->i2clock); // remove for maintouch mutex

	dbg_func_out();
}


/* ------------- Register ---------------*/
/* -------------------------------------------------------------------- */
/* Driver */
/* -------------------------------------------------------------------- */
static int tchkeypt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	dbg_func_in();

	if (!client)
		return 0;

	/* disable worker */
	disable_irq_nosync(client->irq);

#ifdef USE_POWER_CONTROL
	gpio_direction_output(TOUCHPAD_SCL,0);  //low
	gpio_direction_output(TOUCHPAD_SDA,0);  //low

	tchpad_power_control(0);
#endif
	dbg_func_out();

	return 0;
}

static int tchkeypt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	dbg_func_in();

	if (!client)
		return 0;

#ifdef USE_POWER_CONTROL
	tchpad_power_control(1);

	gpio_direction_output(TOUCHPAD_SCL,1);  //high
	gpio_direction_output(TOUCHPAD_SDA,1);  //high
#endif
	enable_irq(client->irq);

	dbg_func_out();
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tchkey_early_suspend(struct early_suspend *handler)
{
	dbg_func_in();

	tchkeypt_suspend(&tchkeyptdata->client->dev);

	dbg_func_out();
}

static void tchkey_late_resume(struct early_suspend *handler)
{
	dbg_func_in();

	tchkeypt_resume(&tchkeyptdata->client->dev);

	dbg_func_out();
}
#endif

static int __devinit tchkeypt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err=0, rc= 0;
	int error = 0;
	dbg_func_in();

	tchkeyptdata = kzalloc (sizeof(struct tchkeyptdata_t),GFP_KERNEL);

	if (tchkeyptdata == NULL) {
		dbg("err kzalloc for tchkey\n");
		err = -ENOMEM;
	}

#ifdef USE_TCHPAD_WORKQUEUE
	tchpad_wq = create_singlethread_workqueue("tchpad_wq");
	if (!tchpad_wq)
	{
		dbg("create_singlethread_workqueue(tchpad_wq) error.\n");
		goto err_exit;//jhseo test
	}
#endif
	

#ifdef USE_POWER_CONTROL
	printk("USE_POWER_CONTROL\n");
	gpio_direction_output(TOUCHPAD_SCL,0);  //low
	gpio_direction_output(TOUCHPAD_SDA,0);  //low

	msleep(5);
#if 0
	l17_1p8v = regulator_get(NULL,  "8058_l17");
	if(IS_ERR(l17_1p8v))
	{
		rc = PTR_ERR(l17_1p8v);
		printk("[TCHKEYPT] regulator_get lvs3b_1p8v fail : 0x%x\n", (unsigned int)l17_1p8v);
	}
	else
	{
		printk("[TCHKEYPT] %s: vreg get level Success (%d)\n", __func__, rc);
	}

	rc = regulator_set_voltage(l17_1p8v, 1800000, 1800000);
	if (rc)
	{
		printk("[TCHKEYPT] %s: vreg set level failed (%d)\n", __func__, rc);
	}
	else
	{
		printk("[TCHKEYPT] %s: vreg set level Success (%d)\n", __func__, rc);
	}
#endif

	tchpad_power_control(1);

	gpio_direction_output(TOUCHPAD_SCL,1);  //high
	gpio_direction_output(TOUCHPAD_SDA,1);  //high
#endif



	rc = gpio_request(TOUCHPAD_RST, "backtouch_rst");
		if (rc) {
			pr_err("gpio_request TOUCHPAD_RST : %d failed, rc=%d\n",TOUCHPAD_RST, rc);
			return -EINVAL;
	}
	

	rc = gpio_direction_output(TOUCHPAD_RST,0);

	msleep(100);

	rc = gpio_direction_output(TOUCHPAD_RST,1);
	msleep(100);


	rc = gpio_request(TCHKEYPT_PS_INT_N, "backtouch_chg");
	if (rc) {
	 	printk("gpio_request TCHKEYPT_PS_INT_N : %d failed, rc=%d\n",TCHKEYPT_PS_INT_N, rc);
		return -EINVAL;
	}  

	rc = gpio_direction_input(TCHKEYPT_PS_INT_N);
	if (rc) {
		printk("gpio_direction_input TCHKEYPT_PS_INT_N : %d failed, rc=%d\n",TCHKEYPT_PS_INT_N, rc);
		return -EINVAL;
	}
	
	
#ifdef USE_FILE_ATTR
	if(!touch_pad_class)
		touch_pad_class=class_create(THIS_MODULE, "touch_rear");

	ts_pad_dev = device_create(touch_pad_class, NULL, 0, NULL, "ts_rear");
	if (IS_ERR(ts_pad_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(ts_pad_dev, &dev_attr_setup) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_setup.attr.name);
#endif

	// 1. initialize mutex
	mutex_init(&tchkeyptdata->i2clock);
	
	// 2. assign i2c client
	tchkeyptdata->client = client;
	i2c_set_clientdata(client, &tchkeyptdata);

	// 3. check available
	if(tchkeyptdata->client == NULL) {
		printk("[ERR] %s : i2c client is NULL\n", __func__);
		goto err_exit;
	}

	if(tchkey_get_available()==0)
	{
		printk("[ERR] %s : i2c id is not valid\n", __func__);
	}
	
	tchkeyptdata->tchkeypt = input_allocate_device();
	if (!tchkeyptdata->tchkeypt) {
		dbg("%s err input allocate device\n",__func__);
		err = -ENOMEM;
		goto err_exit;
	}

	tchkeyptdata->tchkeypt = tchkeyptdata->tchkeypt;
	tchkeyptdata->tchkeypt->name = TCHKEYPT_DRV_NAME;
	tchkeyptdata->tchkeypt->dev.parent = &client->dev;

	set_bit(EV_KEY, tchkeyptdata->tchkeypt->evbit);
	set_bit(EV_ABS, tchkeyptdata->tchkeypt->evbit);
    set_bit(EV_SYN, tchkeyptdata->tchkeypt->evbit);
	set_bit(BTN_TOUCH, tchkeyptdata->tchkeypt->keybit);


#ifdef MULTI_TOUCH
	input_mt_init_slots(tchkeyptdata->tchkeypt, MAX_NUM_FINGER);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_MT_POSITION_X, 0, RESOLUTION_X-1, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_MT_POSITION_Y, 0, RESOLUTION_Y-1, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#endif

#ifdef SINGLE_TOUCH
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_X, 0, RESOLUTION_X-1, 0, 0);
	input_set_abs_params(tchkeyptdata->tchkeypt, ABS_Y, 0, RESOLUTION_Y-1, 0, 0);
#endif




	error = input_register_device(tchkeyptdata->tchkeypt);
	if (error) {
		dbg("%s tchkeypt : Failed to register input device\n",__func__);
		err = -ENOMEM;
		goto err_exit;	}

	input_set_drvdata(tchkeyptdata->tchkeypt, tchkeyptdata);

	
	rc = misc_register(&touch_io);
	if (rc) 
	{
		pr_err("::::::::: can''t register qt602240 misc\n");
	}
	

	tchkeyptdata->client->irq = TCHKEYPT_PS_IRQ;

	//error = request_irq (tchkeyptdata->client->irq,tchkeypt_irq_handler,IRQF_TRIGGER_FALLING,"tchkeypt_ps_irq", tchkeyptdata);
	error = request_irq (tchkeyptdata->client->irq,tchkeypt_irq_handler,IRQF_TRIGGER_LOW,"tchkeypt_ps_irq", tchkeyptdata);

	if (error) {
		dbg("%s irq request error \n", __func__);
		err = -ENOMEM;
		goto err_exit;
	}

	tchkeyptdata->new_keyvalue=0xff;
	tchkeyptdata->PAD_keyvalue=0xff;
	tchkeyptdata->PAD_FUNCTION=0x00;


	INIT_WORK(&tchkeyptdata->work, tchkeypt_work_f);

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("CONFIG_HAS_EARLYSUSPEND\n");

	tchkeyptdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tchkeyptdata->early_suspend.suspend = tchkey_early_suspend;
	tchkeyptdata->early_suspend.resume = tchkey_late_resume;
	register_early_suspend(&tchkeyptdata->early_suspend);
#endif
	dbg_func_out();
	
	return err;

err_exit:
	if (tchkeyptdata->tchkeypt) {
		input_free_device(tchkeyptdata->tchkeypt);
	}

	if (tchkeyptdata != NULL) {
		kfree(tchkeyptdata);
	}

#ifdef USE_POWER_CONTROL
	if(l17_1p8v) {
		regulator_put(l17_1p8v);
	}
#endif
	return -EIO;
}

static void tchkeypt_shutdown(struct i2c_client *client)
{
	u8 tchpwron=TURN_OFF;

	dbg_func_in();

	if(client!=NULL)
	{
		tchkeyptdata->MSM_STATUS=(unsigned int)tchpwron;
	}
	dbg_func_out();
}

static int __devexit tchkeypt_remove(struct i2c_client *client)
{
	int rc = 0;
	dbg_func_in();

	if(client != NULL) kfree(i2c_get_clientdata(client));

#ifdef USE_TCHPAD_WORKQUEUE
	if (tchpad_wq)
		destroy_workqueue(tchpad_wq);
#endif

	rc = misc_register(&touch_io);
	if (rc) 
	{
		pr_err("::::::::: can''t register qt602240 misc\n");
	}

#ifdef USE_POWER_CONTROL
	tchpad_power_control(0);

	if(l17_1p8v) {
		regulator_put(l17_1p8v);
	}
#endif

	dbg_func_out();

	return 0;
}

// FILE IO

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
	u8 cmd[1];
	if((size_t)(*ppos) > 0) return 0;
	if(buf!=NULL)
	{
		nBufSize=strlen(buf);
		if(strncmp(buf, "queue", 5)==0)
		{
			queue_work(tchpad_wq, &tchkeyptdata->work);
		}
		/*if(strncmp(buf, "debug", 5)==0)
		{			
			DebugON=1;	 
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
		if(strncmp(buf, "save", 4)==0)
		{			
			backup_config();	    
		}*/
		if(strncmp(buf, "done", 4)==0)
		{	
			cmd[0] = I2C_DONE_VALUE;
			
			tchkeypt_i2c_addr_write(I2C_DONE_ADDR,&cmd[0],1);
		}
	}
	*ppos +=nBufSize;
	return nBufSize;
}
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return 0; 
}

static const struct i2c_device_id tchkeypt_id[] = {
	{ "tchkeypt", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tchkeypt_id);

static const struct dev_pm_ops i2c_device_tchkeypt_pm_ops = {
	.suspend = tchkeypt_suspend,
	.resume = tchkeypt_resume,
};

static struct i2c_driver tchkeypt_driver = {
	.driver = {
		.name	= TCHKEYPT_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm = &i2c_device_tchkeypt_pm_ops,
	},
	.probe	= tchkeypt_probe,
	.remove	= __devexit_p(tchkeypt_remove),
	.shutdown = tchkeypt_shutdown,
	.id_table = tchkeypt_id,
};

static int __init tchkeypt_init(void)
{
	dbg_func_in();
#if 1
	return i2c_add_driver(&tchkeypt_driver);
#endif
}

static void __exit tchkeypt_exit(void)
{
#if 1
	i2c_del_driver(&tchkeypt_driver);
#endif
}

/* -------------------------------------------------------------------- */

MODULE_AUTHOR("Seo JunHyuk <seo.junhyuk@pantech.com>");
MODULE_DESCRIPTION("TCHKEYPT proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(tchkeypt_init);
module_exit(tchkeypt_exit);

/* -------------------------------------------------------------------- */
