/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_renesas.h"

static struct msm_panel_info pinfo;

#if defined (MIPI_CLOCK_860MBPS)
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
#if 0 // ejkim test clk enable
	/* 720*1280, RGB888, 4 Lane 60 fps 440Mbps video mode */
  /* regulator */
  {0x03, 0x0a, 0x04, 0x00, 0x20},
  /* timing */
  {0xab, 0x8a, 0x18, 0x00, 0x92, 0x97, 0x1b, 0x8c,
   0x0c, 0x03, 0x04, 0xa0},
  /* phy ctrl */
  {0x5f, 0x00, 0x00, 0x10},
  /* strength */
  {0xff, 0x00, 0x06, 0x00},
  /* pll control */
  {0x0, 0xad, 0x01, 0x1a, 0x00, 0x50, 0x48, 0x63,
   0x30, 0x07, 0x01,
   0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
#else //ejkim
  /* regulator */
  {0x03, 0x0a, 0x04, 0x00, 0x20},
  /* timing */
  {0xe3, 0x99, 0x38, 0x00, 0x3e, 0xa4, 0x3a, 0x9b,
   0x3e, 0x03, 0x04, 0xa0},
  /* phy ctrl */
  {0x5f, 0x00, 0x00, 0x10},
  /* strength */
  {0xff, 0x00, 0x06, 0x00},
  /* pll control */
  {0x0, 0xad, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
   0x30, 0x07, 0x01,
   0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },

#endif //ejkim
};

#else // MIPI_CLOCK_860MBPS

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
#ifdef CONFIG_FB_MSM_MDP303
	/* DSI Bit Clock at 500 MHz, 2 lane, RGB888 */
	/* regulator */
	{0x03, 0x01, 0x01, 0x00},
	/* timing   */
	{0xb9, 0x8e, 0x1f, 0x00, 0x98, 0x9c, 0x22, 0x90,
	0x18, 0x03, 0x04},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x00},
	/* strength */
	{0xbb, 0x02, 0x06, 0x00},
	/* pll control */
	{0x00, 0xec, 0x31, 0xd2, 0x00, 0x40, 0x37, 0x62,
	0x01, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0},
#else
	/* DSI_BIT_CLK at 400MHz, 1 lane, RGB888 */
	/* regulator */
	{0x03, 0x01, 0x01, 0x00},
	/* timing   */
	{0xaa, 0x3b, 0x1b, 0x00, 0x52, 0x58, 0x20, 0x3f,
	0x2e, 0x03, 0x04},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x00},
	/* strength */
	{0xee, 0x00, 0x86, 0x00},
	/* pll control */
	{0x40, 0xc7, 0xb0, 0xda, 0x00, 0x50, 0x48, 0x63,
#if defined(RENESAS_FWVGA_TWO_LANE)
	0x30, 0x07, 0x03,
#else
	/* default set to 1 lane */
	0x30, 0x07, 0x07,
#endif
	0x05, 0x14, 0x03, 0x0, 0x0, 0x54, 0x06, 0x10, 0x04, 0x0},
#endif
};

#endif // MIPI_CLOCK_860MBPS

static int __init mipi_video_renesas_fhd_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_renesas_fhd"))
		return 0;
#endif
#if defined(MIPI_CLOCK_860MBPS)
	pinfo.xres = 1080;// kkcho_temp 720;
	pinfo.yres = 1920;// kkcho_temp 1280;
#else

	pinfo.xres = 480;
	pinfo.yres = 864;
#endif
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

#if defined (MIPI_CLOCK_860MBPS)
  	pinfo.lcdc.h_back_porch = 50;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 10;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 4;
    pinfo.lcdc.v_pulse_width = 2;
		
#else //MIPI_CLOCK_860MBPS
		
#ifdef CONFIG_FB_MSM_MDP303
	pinfo.lcdc.h_back_porch = 100;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 20;
	pinfo.lcdc.v_front_porch = 20;
	pinfo.lcdc.v_pulse_width = 1;
	pinfo.clk_rate = 499000000;
#else // CONFIG_FB_MSM_MDP303

#if defined(RENESAS_FWVGA_TWO_LANE)
	pinfo.lcdc.h_back_porch = 400;
#else
	pinfo.lcdc.h_back_porch = 50;
#endif
	pinfo.lcdc.h_front_porch = 50;

#if defined(RENESAS_FWVGA_TWO_LANE)
	pinfo.lcdc.h_pulse_width = 5;
#else
	pinfo.lcdc.h_pulse_width = 20;
#endif

#if defined(RENESAS_FWVGA_TWO_LANE)
	pinfo.lcdc.v_back_porch = 75;
	pinfo.lcdc.v_front_porch = 5;
	pinfo.lcdc.v_pulse_width = 1;
#else
	pinfo.lcdc.v_back_porch = 10;
	pinfo.lcdc.v_front_porch = 10;
	pinfo.lcdc.v_pulse_width = 5;
#endif

#endif // CONFIG_FB_MSM_MDP303

#endif // MIPI_CLOCK_860MBPS


	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
#if defined (MIPI_CLOCK_860MBPS)  // kkcho_temp 
#if defined(CONFIG_SKY_EF51S_BOARD)||defined(CONFIG_SKY_EF51L_BOARD)		// p14682 kobj 120807 ef51L porting
	pinfo.bl_max = 32;
#else
	pinfo.bl_max = 16;
#endif 
#else
	pinfo.bl_max = 100;
#endif
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
#if defined (MIPI_CLOCK_860MBPS)  // kkcho_temp 
    	pinfo.clk_rate = 860000000;
#endif

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE; // shkwak 20120608, TRUE make white blank ..
	pinfo.mipi.hsa_power_stop = TRUE;
#if defined (MIPI_CLOCK_860MBPS)  // kkcho_temp 
	pinfo.mipi.eof_bllp_power_stop = FALSE;
#else
	pinfo.mipi.eof_bllp_power_stop = TRUE;
#endif
	pinfo.mipi.bllp_power_stop = TRUE;

#if defined (MIPI_CLOCK_860MBPS)

	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
   	pinfo.mipi.data_lane0 = TRUE;
  	pinfo.mipi.data_lane1 = TRUE;
   	pinfo.mipi.data_lane2 = TRUE;
   	pinfo.mipi.data_lane3 = TRUE;


  	pinfo.mipi.t_clk_post = 0x20;// 2036 = 0x7f4 was tried but failed.. 
	pinfo.mipi.t_clk_pre = 0x36;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = 0;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 57; //60;

	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.tx_eot_append = TRUE;
#else //MIPI_CLOCK_860MBPS

#ifdef CONFIG_FB_MSM_MDP303
	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format =  DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2F;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.dlane_swap = 0x01;
	pinfo.mipi.tx_eot_append = 0x01;

	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.tx_eot_append = TRUE;

#else
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
#if defined(RENESAS_FWVGA_TWO_LANE)
	pinfo.mipi.data_lane1 = TRUE;
#else
	pinfo.mipi.data_lane1 = FALSE;
#endif
	pinfo.mipi.t_clk_post = 0x03;
	pinfo.mipi.t_clk_pre = 0x24;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
#endif // CONFIG_FB_MSM_MDP303

#endif //MIPI_CLOCK_860MBPS
	pinfo.mipi.esc_byte_ratio = 4;  // 20120621, kkcho, added after 1023-patch

	ret = mipi_renesas_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_renesas_fhd_init);
