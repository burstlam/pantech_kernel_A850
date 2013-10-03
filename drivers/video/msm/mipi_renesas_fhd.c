/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_renesas.h"
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/system.h>

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE  0

#define NOP()	do {asm volatile ("NOP");} while(0);
#define DELAY_3NS() do { \
    asm volatile ("NOP"); \
    asm volatile ("NOP"); \
    asm volatile ("NOP");} while(0);

#define LCD_DEBUG_MSG

#ifdef LCD_DEBUG_MSG
#define ENTER_FUNC()        printk(KERN_INFO "[SKY_LCD] +%s \n", __FUNCTION__);
#define EXIT_FUNC()         printk(KERN_INFO "[SKY_LCD] -%s \n", __FUNCTION__);
#define ENTER_FUNC2()       printk(KERN_ERR "[SKY_LCD] +%s\n", __FUNCTION__);
#define EXIT_FUNC2()        printk(KERN_ERR "[SKY_LCD] -%s\n", __FUNCTION__);
#define PRINT(fmt, args...) printk(KERN_INFO fmt, ##args)
#define DEBUG_EN 1
#else
#define PRINT(fmt, args...)
#define ENTER_FUNC2()
#define EXIT_FUNC2()
#define ENTER_FUNC()
#define EXIT_FUNC()
#define DEBUG_EN 0
#endif

//#define FEATURE_TP_SAMPLE

#if CONFIG_BOARD_VER >= CONFIG_WS20   // 20120502, [LS5]kkcho, PM_GPIO->MSM_GPIO
#define SKY_LCD_VCI_EN_PIN_CHANGE  
#else
#if defined(CONFIG_SKY_EF51S_BOARD)||defined(CONFIG_SKY_EF51L_BOARD)		// p14682 kobj 120807 ef51L porting
#define SKY_LCD_VCI_EN_PIN_CHANGE  
#define SKY_LCD_SINGLE_WIRE_LB_CON					// p14682 kobj 120809 add bl con 
#define LCD_BL_MAX 32								// p14682 kobj 120809 add bl con 
#endif
#endif

extern int gpio_lcd_mipi_reset, gpio_lcd_bl_en;/* gpio43 :gpio_lcd_mipi_reset, gpio16:lgpio_lcd_bl_en , */

#ifdef SKY_LCD_SINGLE_WIRE_LB_CON					// p14682 kobj 120809 add bl con 
extern int gpio_lcd_bl_ctl;
#endif // p14682 kobj 120809 add bl con 

#ifdef SKY_LCD_VCI_EN_PIN_CHANGE
#define LCD_VCI_EN 82
#else
extern int gpio_lcd_vci_en;
#endif

static struct msm_panel_common_pdata *mipi_renesas_pdata;

static struct dsi_buf renesas_tx_buf;
static struct dsi_buf renesas_rx_buf;

struct lcd_state_type {
    boolean disp_powered_up;
    boolean disp_initialized;
    boolean disp_on;
#ifdef CONFIG_LCD_CABC_CONTROL
		int acl_flag;
#endif
};

static struct lcd_state_type renesas_state = { 0, };


char extcctl[5]     = {0xdf, 0x55,0xaa,0x52,0x08};
char eics[9]        = {0xb0, 0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char cas[9]         = {0xb1, 0x01,0x82,0x04,0x00,0x00,0x00,0x00,0x00};
char ifs[9]         = {0xb2, 0x00,0x00,0x00,0x00,0x01,0x00,0x16,0x00};
char dfs[5]         = {0xb3, 0x00,0x3a,0x00,0x0a};
char dfs_1[9]       = {0xb4, 0x02,0xcf,0x04,0xff,0x00,0x00,0x00,0xa4};
char dfs_2[9]       = {0xb5, 0x00,0x0d,0x00,0x00,0x00,0x04,0x00,0x00}; //PWM 6Khz
//char dfs_2[9]       = {0xb5, 0x00,0x06,0x00,0x00,0x00,0x04,0x00,0x00};  //PWM 13Khz
char pc_1[9]        = {0xb6, 0x49,0x07,0x10,0x00,0x00,0x53,0x44,0x00};
char dc[9]          = {0xb8, 0x73,0x0a,0x91,0x1e,0x00,0x08,0xb5,0xb5};
char vdcs[9]        = {0xb9, 0x00,0x01,0x37,0x00,0x00,0x00,0x00,0x00};
char gcev[13]       = {0xc0, 0x68,0xff,0x68,0xff,0x80,0xff,0x5b,0xff,0x5b,0xff,
                             0x71,0xff};
char gcpr[17]       = {0xc1, 0x71,0x7b,0x8b,0xa6,0xcd,0xc9,0x83,0xb0,0x64,0x82,
                             0x55,0x8e,0xa0,0xb4,0xc8,0xdc};
char gcpg[17]       = {0xc2, 0x71,0x7b,0x8b,0xa6,0xcd,0xc9,0x83,0xb0,0x64,0x82,
                             0x55,0x8e,0xa0,0xb4,0xc8,0xdc};
char gcpb[17]       = {0xc3, 0x85,0x8d,0x9b,0xb1,0xd4,0xce,0x8a,0xb5,0x68,0x85,
                             0x57,0x94,0xaa,0xbe,0xd2,0xe6};
char gcnr[17]       = {0xc4, 0x63,0x6d,0x7d,0x97,0xbd,0xb9,0x61,0x8d,0x3f,0x5d,
                             0x42,0x7a,0x96,0xaa,0xbe,0xd2};
char gcng[17]       = {0xc5, 0x63,0x6d,0x7d,0x97,0xbd,0xb9,0x61,0x8d,0x3f,0x5d,
                             0x42,0x7a,0x96,0xaa,0xbe,0xd2};
char gcnb[17]       = {0xc6, 0x77,0x7e,0x8c,0xa2,0xc4,0xbd,0x68,0x92,0x44,0x60,
                             0x44,0x80,0x96,0xaa,0xbe,0xd2};
//char pits[17]       = {0xc8, 0x11,0x18,0x0d,0x0d,0x28,0x12,0x00,0x00,0x00,0x00,
//                             0x00,0x00,0x00,0x00,0x00,0x00};
//char pits[17]       = {0xc8, 0x10,0x16,0x10,0x10,0x22,0x00,0x00,0x00,0x00,0x00,
//                             0x00,0x00,0x00,0x00,0x00,0x00};
char pits[17]       = {0xc8, 0x11,0x19,0x04,0x0d,0x28,0x08,0x00,0x04,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00};
char cscs_1[17]     = {0xca, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00};
char cscs_2[17]     = {0xcb, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00};

char sleep_out[2]   = {0x11, 0x00};
char disp_on[2]     = {0x29, 0x00};
char sleep_in[2]    = {0x10, 0x00};
char disp_off[2]    = {0x28, 0x00};



static struct dsi_cmd_desc renesas_display_on_cmds[] = {
#if defined(FEATURE_RENESAS_FHD)
    {DTYPE_DCS_WRITE, 1, 0, 0, 17, sizeof(disp_on), disp_on},
	    {DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(sleep_out), sleep_out}
    //{DTYPE_DCS_WRITE, 1, 0, 0, 17, sizeof(disp_on), disp_on}

#else //FEATURE_RENESAS_FHD
#ifndef FEATURE_TP_SAMPLE	
    {DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_out), sleep_out},
#endif    
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_on), disp_on}
#endif //FEATURE_RENESAS_FHD
};

static struct dsi_cmd_desc renesas_display_off_cmds[] = {
#if defined(FEATURE_RENESAS_FHD)
    {DTYPE_DCS_WRITE, 1, 0, 0, 17, sizeof(disp_off), disp_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 68, sizeof(sleep_in), sleep_in}
#else //FEATURE_RENESAS_FHD
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_off), disp_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_in), sleep_in}
#endif //FEATURE_RENESAS_FHD
};



static int mipi_renesas_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	
	if (renesas_state.disp_initialized == false) {
		 /*20120625, kkcho, Must be the preservation of order : reset->vci*/
           	gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_LOW_VALUE); 
           	msleep(10);
		gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_HIGH_VALUE);
		msleep(10);
#ifdef SKY_LCD_VCI_EN_PIN_CHANGE 
		gpio_set_value(LCD_VCI_EN, GPIO_HIGH_VALUE);
		msleep(1);
#else
        	gpio_set_value_cansleep(gpio_lcd_vci_en, 1);
#endif

#if !defined(FEATURE_RENESAS_FHD)		
		mipi_dsi_cmds_tx(mfd, &renesas_tx_buf, renesas_display_init_cmds,
				ARRAY_SIZE(renesas_display_init_cmds));
#endif
		renesas_state.disp_initialized = true;
	}

#if defined(FEATURE_RENESAS_FHD)
		
		printk(KERN_ERR"[SKY_LCD] mipi_renesas_lcd_on mipi_reset start\n");

        msleep(10);
        gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_LOW_VALUE); 
        msleep(10);
		gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_HIGH_VALUE);
        msleep(10);
		printk(KERN_ERR"[SKY_LCD] mipi_renesas_lcd_on mipi_reset end\n");
	
#endif

	mipi_dsi_cmds_tx(mfd, &renesas_tx_buf, renesas_display_on_cmds,
			ARRAY_SIZE(renesas_display_on_cmds));
      renesas_state.disp_on = true;

#if defined(FEATURE_RENESAS_FHD)
	  msleep(200); //shkwak, FEATURE_RENESAS_FHD
#endif


	EXIT_FUNC2();
	return 0;
}

static int mipi_renesas_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (renesas_state.disp_on == true) {
		//gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_LOW_VALUE);
		//msleep(5);
		//gpio_set_value_cansleep(gpio_lcd_mipi_reset, GPIO_HIGH_VALUE);
		//msleep(5);
        mipi_set_tx_power_mode(0);
		mipi_dsi_cmds_tx(mfd, &renesas_tx_buf, renesas_display_off_cmds,
				ARRAY_SIZE(renesas_display_off_cmds));
		renesas_state.disp_on = false;
		renesas_state.disp_initialized = false;
        mipi_set_tx_power_mode(1);	
	}
	
    EXIT_FUNC2();
	return 0;
}

static int first_enable = 0;
static int prev_bl_level = 0;


static void mipi_renesas_set_backlight(struct msm_fb_data_type *mfd)
{
#ifdef SKY_LCD_SINGLE_WIRE_LB_CON					// p14682 kobj 120809 add bl con 
	int cnt, bl_level;	
	unsigned long flags;

	if (prev_bl_level == mfd->bl_level)
		return;

	ENTER_FUNC2();

	bl_level=mfd->bl_level;

	mipi_set_tx_power_mode(0);

	printk(KERN_ERR"mipi_renesas_set_backlight bl_level =%d \n",bl_level);

	if (bl_level == prev_bl_level) 
	{
		return;
	} 
	else 
	{
		if (bl_level == 0) {
			gpio_set_value_cansleep(gpio_lcd_bl_ctl, GPIO_LOW_VALUE);
			usleep(300);      // Disable hold time
			gpio_set_value_cansleep(gpio_lcd_bl_en, GPIO_HIGH_VALUE);		
			msleep(5);			
			first_enable  = 0;
		} 
		else 
		{
			if (prev_bl_level == 0) {
				msleep(100); 
				gpio_set_value_cansleep(gpio_lcd_bl_en, GPIO_HIGH_VALUE);
				msleep(5); 

				gpio_set_value_cansleep(gpio_lcd_bl_ctl, GPIO_HIGH_VALUE);
				if (first_enable == 0) {
					first_enable = 1;
					local_save_flags(flags);
					local_irq_disable();
					udelay(10);	// T_EN
					mdelay(3);	// T_SS
					local_irq_restore(flags);
				} else {
					udelay(300);      // Turn on time
				}
			}

			if (prev_bl_level < bl_level) {
				cnt = LCD_BL_MAX - bl_level;
				cnt += prev_bl_level;
			} else {
				cnt = prev_bl_level - bl_level;
			}
		
		
			//PRINT("[LIVED] cnt=%d\n", cnt);
			while (cnt) {
				local_save_flags(flags);
				local_irq_disable();
				gpio_set_value_cansleep(gpio_lcd_bl_ctl, GPIO_LOW_VALUE);
				udelay(3);//DELAY_3NS();//udelay(3);      // Turn off time
				gpio_set_value_cansleep(gpio_lcd_bl_ctl, GPIO_HIGH_VALUE);
			    local_irq_restore(flags);
				udelay(10);      // Turn on time
				//PRINT("[LIVED] (2) cnt=%d!\n", cnt);
				cnt--;
			}
			
			//PRINT("[LIVED] count=%d\n", cnt);


		}
			
		prev_bl_level = bl_level;
		
	}

	mipi_set_tx_power_mode(1);

#ifdef CONFIG_F_PREVENT_RESET_DURING_BOOTUP
	mfd->backlight_on_after_bootup =1;	
#endif

	EXIT_FUNC2(); 

#else	//SKY_LCD_SINGLE_WIRE_LB_CON
	int bl_level;

	if (prev_bl_level == mfd->bl_level)
		return;

	ENTER_FUNC2();

	bl_level = (mfd->bl_level *255)/16;


	printk(KERN_ERR"mipi_renesas_set_backlight bl_level =%d \n",bl_level);

	if(first_enable == 0)
	{
	    gpio_set_value_cansleep(gpio_lcd_bl_en, GPIO_HIGH_VALUE);
	    first_enable  = 1;
	    
	}
	
	mipi_set_tx_power_mode(0);

	prev_bl_level = mfd->bl_level;
	mipi_set_tx_power_mode(1);

	if(bl_level == 0)
	{
	      gpio_set_value_cansleep(gpio_lcd_bl_en, GPIO_LOW_VALUE);
	      first_enable = 0;
	}
	EXIT_FUNC2();    
#endif	//SKY_LCD_SINGLE_WIRE_LB_CON
}

//shkwak 20120608, add for temp build
void ce_contol(struct msm_fb_data_type *mfd, int count)
{
	printk(KERN_ERR"[LCD] %s+, do nothing\n", __func__);
}
void cabc_contol(struct msm_fb_data_type *mfd, int state)
{
	printk(KERN_ERR"[LCD] %s+, do nothing\n", __func__);
}
void SKY_LCD_CE_CASE_SET(struct msm_fb_data_type *mfd, uint32_t CEcase)
{
	printk(KERN_ERR"[LCD] %s+, do nothing\n", __func__);
}

static int __devinit mipi_renesas_lcd_probe(struct platform_device *pdev)
{
    if (pdev->id == 0) {
        mipi_renesas_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_renesas_lcd_probe,
	.driver = {
		.name   = "mipi_renesas",
	},
};

static struct msm_fb_panel_data renesas_panel_data = {
       .on             = mipi_renesas_lcd_on,
       .off            = mipi_renesas_lcd_off,
       .set_backlight  = mipi_renesas_set_backlight,
};

static int ch_used[3];

int mipi_renesas_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_renesas", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	renesas_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &renesas_panel_data,
		sizeof(renesas_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_renesas_lcd_init(void)
{
    ENTER_FUNC2();

    renesas_state.disp_powered_up = true;

    mipi_dsi_buf_alloc(&renesas_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&renesas_rx_buf, DSI_BUF_SIZE);

    EXIT_FUNC2();

    return platform_driver_register(&this_driver);
}

module_init(mipi_renesas_lcd_init);

