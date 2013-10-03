//*****************************************************************************
//*****************************************************************************
//  FILENAME: Driver.h
//  TrueTouch Host Emulator Version Information: 2.1.681
//  TrueTouch Firmware Version Information: 1.1.360995
//
//  DESCRIPTION: This file contains configuration values.
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress Semiconductor 2012. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
//-----------------------------------------------------------------------------
/* Touchscreen Parameters Endianess (Endianess: 0:Little; 1:Big)*/
static const uint8_t cyttsp4_param_endianess = 0;

/* Touchscreen Parameters */
static const uint8_t cyttsp4_param_regs[] = {
/*	Value	Name	*/
	0xFC, 0x05,  /* CONFIG_DATA_SIZE */
	0xFC, 0x05,  /* CONFIG_DATA_MAX_SIZE */
	0x4C, 0x00, 0x00, 0x00,  /* SDK_CTRL_CFG_SIZE */
	0x25, 0x00,  /* CONFIG_VER */
	0x01,  /* PANELID_ENABLE */
	0x30,  /* IMO_FREQ_MHZ */
	0xF6, 0x19,  /* X_LEN_PHY */
	0xEA, 0x2D,  /* Y_LEN_PHY */
	0x14,  /* HST_MODE0 */
	0x07,  /* ACT_DIST0 */
	0x00,  /* SCAN_TYP0 */
	0x08,  /* ACT_INTRVL0 */
	0x03,  /* ACT_LFT_INTRVL0 */
	0x00,  /* Reserved21 */
	0x32, 0x00,  /* LP_INTRVL0 */
	0xB8, 0x0B,  /* TCH_TMOUT0 */
	0x00,  /* PWR_CFG */
	0x00,  /* Reserved27 */
	0x32,  /* INT_PULSE_DATA */
	0x28,  /* OPMODE_CFG */
	0xF4, 0x01,  /* HANDSHAKE_TIMEOUT */
	0xF5, 0x01,  /* ESD_COUNTER_CFG */
	0x1E,  /* TIMER_CAL_INTERVAL */
	0x00,  /* Reserved35 */
	0x00, 0x00,  /* RP2P_MIN */
	0xB8, 0x0B,  /* ILEAK_MAX */
	0x2C, 0x01,  /* RFB_P2P */
	0x2C, 0x01,  /* RFB_EXT */
	0x00,  /* IDACOPEN_LOW */
	0x00,  /* IDACOPEN_HIGH */
	0x00,  /* IDACOPEN_BUTTON_LOW */
	0x00,  /* IDACOPEN_BUTTON_HIGH */
	0x00,  /* GIDAC_OPEN */
	0x00,  /* GAIN_OPEN */
	0x00,  /* GIDAC_BUTTON_OPEN */
	0x00,  /* GAIN_BUTTON_OPEN */
	0x00,  /* POST_CFG */
	0x00,  /* GESTURE_CFG */
	0x00,  /* GEST_EN0 */
	0x00,  /* GEST_EN1 */
	0x00,  /* GEST_EN2 */
	0x00,  /* GEST_EN3 */
	0x00,  /* GEST_EN4 */
	0x00,  /* GEST_EN5 */
	0x00,  /* GEST_EN6 */
	0x00,  /* GEST_EN7 */
	0x00,  /* GEST_EN8 */
	0x00,  /* GEST_EN9 */
	0x00,  /* GEST_EN10 */
	0x00,  /* GEST_EN11 */
	0x00,  /* GEST_EN12 */
	0x00,  /* GEST_EN13 */
	0x00,  /* GEST_EN14 */
	0x00,  /* GEST_EN15 */
	0x00,  /* GEST_EN16 */
	0x00,  /* GEST_EN17 */
	0x00,  /* GEST_EN18 */
	0x00,  /* GEST_EN19 */
	0x00,  /* GEST_EN20 */
	0x00,  /* GEST_EN21 */
	0x00,  /* GEST_EN22 */
	0x00,  /* GEST_EN23 */
	0x00,  /* GEST_EN24 */
	0x00,  /* GEST_EN25 */
	0x00,  /* GEST_EN26 */
	0x00,  /* GEST_EN27 */
	0x00,  /* GEST_EN28 */
	0x00,  /* GEST_EN29 */
	0x00,  /* GEST_EN30 */
	0x00,  /* GEST_EN31 */
	0x00,  /* ACT_DIST2 */
	0x04,  /* ACT_DIST_TOUCHDOWN */
	0x23,  /* ACT_DIST_LIFTOFF */
	0x0C,  /* ACT_DIST_COUNTER */
	0x0A,  /* ACT_DIST_Z_THRESHOLD */
	0x00,  /* EXTERN_SYNC */
	0x00, 0x00, 0x00, 0x00,  /* Reserved92 */
	0x18, 0x00, 0x00, 0x00,  /* GRIP_CFG_SIZE */
	0x06, 0x00,  /* GRIP_XEDG_A */
	0x06, 0x00,  /* GRIP_XEDG_B */
	0x06, 0x00,  /* GRIP_XEXC_A */
	0x06, 0x00,  /* GRIP_XEXC_B */
	0x06, 0x00,  /* GRIP_YEDG_A */
	0x06, 0x00,  /* GRIP_YEDG_B */
	0x06, 0x00,  /* GRIP_YEXC_A */
	0x06, 0x00,  /* GRIP_YEXC_B */
	0x00,  /* GRIP_FIRST_EXC */
	0x00,  /* GRIP_EXC_EDGE_ORIGIN */
	0x00, 0x00,  /* Reserved118 */
	0x70, 0x00, 0x00, 0x00,  /* TRUETOUCH_CFG_SIZE */
	0x64, 0x00, 0x00, 0x00,  /* MAX_SELF_SCAN_INTERVAL */
	0x64, 0x00, 0x00, 0x00,  /* MAX_MUTUAL_SCAN_INTERVAL */
	0x64, 0x00, 0x00, 0x00,  /* MAX_BALANCED_SCAN_INTERVAL */
	0x0A, 0x00, 0x00, 0x00,  /* SELF_Z_THRSH */
	0x01, 0x00, 0x00, 0x00,  /* SELF_Z_MODE */
	0x01, 0x00, 0x00, 0x00,  /* SMART_SCAN_ENABLE */
	0x01, 0x00, 0x00, 0x00,  /* T_COMP_ENABLE */
	0xD0, 0x07, 0x00, 0x00,  /* T_COMP_INTERVAL */
	0x64, 0x00, 0x00, 0x00,  /* T_COMP_RECAL_MUTUAL_SENSOR_LIMIT */
	0x55, 0x00, 0x00, 0x00,  /* T_COMP_RECAL_MUTUAL_HIGH */
	0xB0, 0xFF, 0xFF, 0xFF,  /* T_COMP_RECAL_MUTUAL_LOW */
	0x0F, 0x00, 0x00, 0x00,  /* T_COMP_RECAL_SELF_SENSOR_LIMIT */
	0x2C, 0x01, 0x00, 0x00,  /* T_COMP_RECAL_SELF_HIGH */
	0x9C, 0xFF, 0xFF, 0xFF,  /* T_COMP_RECAL_SELF_LOW */
	0x01, 0x00, 0x00, 0x00,  /* CHARGER_ARMOR_ENABLE */
	0x00, 0x00,  /* AFH_ENABLE */
	0x00, 0x00,  /* AFH_SELF_ENABLE */
	0x0C, 0x00, 0x00, 0x00,  /* AFH_LISTENING_SCAN_COUNT */
	0x08, 0x00, 0x00, 0x00,  /* AFH_LISTEN_SCAN_CYCLE_REPEATS */
	0x00, 0x00, 0x00, 0x00,  /* AFH_LISTEN_SCAN_CONFIG */
	0x64, 0x00, 0x00, 0x00,  /* CA_BLOCK_NOISE_THRESHOLD */
	0x03, 0x00, 0x00, 0x00,  /* CA_BLOCK_NOISE_HYSTERESIS */
	0xDC, 0x05, 0x00, 0x00,  /* CA_DEFAULT_REVERT_TIME */
	0x01, 0x00,  /* CA_SMART_H2O_REJECT */
	0x00, 0x00,  /* CA_HOST_CONTROLLED_CHARGER */
	0xA0, 0x00,  /* T_COMP_BUTTON_MUTUAL_HIGH */
	0xC4, 0xFF,  /* T_COMP_BUTTON_MUTUAL_LOW */
	0xA0, 0x00,  /* T_COMP_BUTTON_SELF_HIGH */
	0xC4, 0xFF,  /* T_COMP_BUTTON_SELF_LOW */
	0x14, 0x00, 0x00, 0x00,  /* CA_NUM_SUB_CONV_BASE_SELF */
	0x08, 0x00, 0x00, 0x00,  /* CA_ALT_NUM_SUB_CONV_SELF */
	0x00,  /* BTN_SCAN_CFG */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00,  /* Reserved233 */
	0x0B,  /* CA_ALT_NUM_SUB_CONV_MUTUAL */
	0x00,  /* CA_ALT_ACQUISITION_FLAGS */
	0x56,  /* AFH_ALT1_TX_PERIOD */
	0x2B,  /* AFH_ALT1_SCALE_MUT */
	0x64,  /* AFH_ALT1_SCALE_SELF */
	0x0F,  /* AFH_ALT1_TX_VOLTAGE */
	0x02,  /* AFH_ALT1_TX_PULSES */
	0x45,  /* AFH_ALT2_TX_PERIOD */
	0x2B,  /* AFH_ALT2_SCALE_MUT */
	0x64,  /* AFH_ALT2_SCALE_SELF */
	0x0F,  /* AFH_ALT2_TX_VOLTAGE */
	0x02,  /* AFH_ALT2_TX_PULSES */
	0x2A,  /* GEST_CFG_SIZE */
	0x01,  /* PAN_ACT_DSTX */
	0x12,  /* PAN_ACT_DSTY */
	0x12,  /* ZOOM_ACT_DSTX */
	0x12,  /* ZOOM_ACT_DSTY */
	0x23,  /* FLICK_ACT_DISTX */
	0x23,  /* FLICK_ACT_DISTY */
	0x50,  /* FLICK_TIME */
	0x02,  /* ST_DEBOUNCE */
	0x03,  /* MT_DEBOUNCE_PAN */
	0x02,  /* MT_DEBOUNCE_ZOOM */
	0x0A,  /* MT_DEBOUNCE_P2Z */
	0x14,  /* ROT_DEBOUNCE */
	0x02,  /* COMPL_DEBOUNCE */
	0x28, 0x00,  /* MT_TIMEOUT */
	0x32,  /* ST_DBLCLK_RMAX */
	0x1E,  /* ST_CLICK_DISTX */
	0x1E,  /* ST_CLICK_DISTY */
	0x00,  /* Reserved271 */
	0xC8, 0x00,  /* MT_CLICK_TMAX */
	0x14, 0x00,  /* MT_CLICK_TMIN */
	0xC8, 0x00,  /* ST_CLICK_TMAX */
	0x14, 0x00,  /* ST_CLICK_TMIN */
	0xC8, 0x00,  /* ST_DBLCLK_TMAX */
	0x14, 0x00,  /* ST_DBLCLK_TMIN */
	0xF0,  /* GESTURE_GROUP_MASK */
	0x28,  /* GESTURE_GROUP1_START */
	0x29,  /* GESTURE_GROUP1_END */
	0x30,  /* GESTURE_GROUP2_START */
	0x3F,  /* GESTURE_GROUP2_END */
	0x48,  /* GESTURE_GROUP3_START */
	0x49,  /* GESTURE_GROUP3_END */
	0x90,  /* GESTURE_GROUP4_START */
	0x9F,  /* GESTURE_GROUP4_END */
	0x00, 0x00, 0x00,  /* Reserved293 */
	0x1C, 0x00, 0x00, 0x00,  /* XY_FILT_CFG_SIZE */
	0xF0, 0x00, 0x00, 0x00,  /* XY_FILTER_MASK */
	0x03, 0x00, 0x00, 0x00,  /* XY_FILT_IIR_COEFF */
	0x02, 0x00, 0x00, 0x00,  /* XY_FILT_Z_IIR_COEFF */
	0x3C,  /* XY_FILT_XY_FAST_THR */
	0x1E,  /* XY_FILT_XY_SLOW_THR */
	0x00,  /* XY_FILT_IIR_FAST_COEFF */
	0x00,  /* Reserved315 */
	0xF0, 0x00, 0x00, 0x00,  /* XY_FILTER_MASK_CA */
	0x02, 0x00, 0x00, 0x00,  /* XY_FILT_IIR_COEFF_CA */
	0x02, 0x00, 0x00, 0x00,  /* XY_FILT_Z_IIR_COEFF_CA */
	0x00,  /* XY_FILT_XY_FAST_THR_CA */
	0x00,  /* XY_FILT_XY_SLOW_THR_CA */
	0x01,  /* XY_FILT_IIR_FAST_COEFF_CA */
	0x00,  /* Reserved331 */
	0x00,  /* XY_FILT_ADAPTIVE_IIR_FILTER */
	0x0C,  /* XY_FILT_ADAPTIVE_IIR_FILTER_DISTANCE */
	0x00,  /* XY_FILT_TOUCH_SIZE_IIR_COEFF */
	0x00,  /* XY_FILT_TOUCH_SIZE_HYST */
	0x00,  /* XY_FILT_TOUCH_ORIENTATION_IIR_COEFF */
	0x00,  /* XY_FILT_TOUCH_ORIENTATION_HYST */
	0x01,  /* XY_FILT_TOUCH_SCALLOPING_ENABLE */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00,  /* Reserved339 */
	0x08, 0x00, 0x00, 0x00,  /* FINGER_ID_CFG_SIZE */
	0x00, 0x00, 0x00, 0x00,  /* Reserved352 */
	0x96, 0x00,  /* MAX_FINGER_VELOCITY */
	0x2C, 0x01,  /* MAX_FINGER_VELOCITY_CA */
	0x00,  /* LIFTOFF_DEBOUNCE */
	0x01,  /* LIFTOFF_DEBOUNCE_CA */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00,  /* Reserved362 */
	0x18, 0x00, 0x00, 0x00,  /* CENTROID_SH_CFG_SIZE */
	0x14, 0x00, 0x00, 0x00,  /* STYLUS_THRSH */
	0x05, 0x00, 0x00, 0x00,  /* STYLUS_HYST */
	0xFF, 0x00, 0x00, 0x00,  /* S2F_THRESHOLD */
	0x00, 0x00, 0x00, 0x00,  /* HOVER_THRSH */
	0x00, 0x00, 0x00, 0x00,  /* HOVER_HYST */
	0x00, 0x00, 0x00, 0x00,  /* Reserved392 */
	0x20, 0x00, 0x00, 0x00,  /* ID_COORDS_CFG_SIZE */
	0x02,  /* LRG_OBJ_CFG */
	0x18,  /* FINGER_THRESH_MUTUAL */
	0x0A,  /* FINGER_THR_MUT_HYST */
	0x09,  /* FINGER_THRESH_SELF */
	0x0F,  /* INNER_EDGE_GAIN */
	0x09,  /* OUTER_EDGE_GAIN */
	0xD0, 0x02,  /* X_RESOLUTION */
	0x00, 0x05,  /* Y_RESOLUTION */
	0x01,  /* SENSOR_ASSIGNMENT */
	0x50,  /* Z_SCALING */
	0x01,  /* RX_LINE_FILTER */
	0x28,  /* RX_LINE_FILTER_THRESHOLD */
	0x02,  /* RX_LINE_FILTER_DEBOUNCE */
	0x01,  /* BYPASS_THRESHOLD_GAIN */
	0x01,  /* BYPASS_THRESHOLD_EDGE_GAIN */
	0x50,  /* MAX_FAT_FINGER_SIZE */
	0x10,  /* MIN_FAT_FINGER_SIZE */
	0x03,  /* MAX_FAT_FINGER_SIZE_HYST */
	0x03,  /* MIN_FAT_FINGER_SIZE_HYST */
	0x20,  /* FAT_FINGER_COEFF */
	0x28,  /* FAT_FINGER_SD_THRESHOLD */
	0x20,  /* FAT_FINGER_COEFF_SD */
	0x02,  /* MULTI_TOUCH_DEBOUNCE */
	0x02,  /* MULTI_TOUCH_DEBOUNCE_CA */
	0x28,  /* FINGER_THRESHOLD_MUTUAL_CA */
	0x0A,  /* FINGER_THRESH_SELF_CA */
	0x03,  /* SIZE_ORIENTATION_ENABLE */
	0x50,  /* MAJOR_AXIS_OFFSET */
	0x0E,  /* MAJOR_AXIS_SCALE */
	0x50,  /* MINOR_AXIS_OFFSET */
	0x0E,  /* MINOR_AXIS_SCALE */
	0x00,  /* CLIPPING_X_LOW */
	0x00,  /* CLIPPING_X_HIGH */
	0x00,  /* CLIPPING_Y_LOW */
	0x00,  /* CLIPPING_Y_HIGH */
	0x00,  /* CLIPPING_BOUNDARY_REMOVE */
	0x00,  /* Reserved438 */
	0x05,  /* COEF_EDGE_ATTRACTION */
	0x70, 0x00,  /* WIDTH_CORNER_DIAG1 */
	0x97, 0x00,  /* WIDTH_CORNER_DIAG2 */
	0x6E, 0xFF,  /* WIDTH_CORNER_DIAG3 */
	0xC2, 0x01,  /* WIDTH_CORNER_DIAG4 */
	0xD7, 0xFF,  /* WIDTH_CORNER_DIAG5 */
	0xCF, 0xFF,  /* WIDTH_CORNER_DIAG6 */
	0x9C, 0xFF,  /* WIDTH_CORNER_DIAG7 */
	0xD9, 0xFF,  /* WIDTH_CORNER_DIAG8 */
	0xDC, 0xFF,  /* WIDTH_CORNER_DIAG9 */
	0xDC, 0xFF,  /* WIDTH_CORNER_DIAG10 */
	0x4F, 0xFF,  /* WIDTH_CORNER_DIAG11 */
	0x07, 0x00,  /* WIDTH_CORNER_PERP1 */
	0x91, 0xFF,  /* WIDTH_CORNER_PERP2 */
	0x14, 0xFF,  /* WIDTH_CORNER_PERP3 */
	0xB3, 0x00,  /* WIDTH_CORNER_PERP4 */
	0x01,  /* NOISE_REJECTION_3x3_FILTER_SCALE */
	0x02,  /* NOISE_REJECTION_3x3_FILTER_SCALE_CA */
	0x01,  /* DIRECT_XY_ENABLE */
	0x01,  /* CALC_THRESHOLD */
	0x00, 0x00,  /* JIG_MODE_CONFIG */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* Reserved476 */
	0x0B, 0x00, 0x00, 0x00,  /* BTN_CFG_SIZE */
	0x64, 0x00,  /* BTN_THRSH_MUT_0 */
	0x64, 0x00,  /* BTN_THRSH_MUT_1 */
	0x64, 0x00,  /* BTN_THRSH_MUT_2 */
	0x64, 0x00,  /* BTN_THRSH_MUT_3 */
	0x14,  /* BTN_HYST_MUT */
	0x00,  /* Reserved497 */
	0x00,  /* Reserved498 */
	0x00,  /* Reserved499 */
	0x32, 0x00,  /* BTN_THRSH_SELF */
	0x64, 0x00,  /* BTN_THRSH_SELF_1 */
	0x64, 0x00,  /* BTN_THRSH_SELF_2 */
	0x64, 0x00,  /* BTN_THRSH_SELF_3 */
	0x0A,  /* BTN_HYST_SELF */
	0x00,  /* Reserved509 */
	0x00,  /* Reserved510 */
	0x00,  /* Reserved511 */
	0x1A, 0x00, 0x00, 0x00,  /* RAW_PROC_CFG_SIZE */
	0x73, 0x77,  /* RAW_FILTER_MASK */
	0x00,  /* RAW_FILT_IIR_COEFF_MUTUAL */
	0x05,  /* RAW_FILT_IIR_THRESHOLD_MUTUAL */
	0x00,  /* RAW_FILT_IIR_COEFF_SELF */
	0x05,  /* RAW_FILT_IIR_THRESHOLD_SELF */
	0x00,  /* RAW_FILT_IIR_COEFF_BALANCED */
	0x14,  /* RAW_FILT_IIR_THRESHOLD_BALANCED */
	0x00,  /* RAW_FILT_IIR_COEFF_BUTTONS */
	0x14,  /* RAW_FILT_IIR_THRESHOLD_BUTTONS */
	0x0A,  /* CMF_THR_MUT */
	0x23,  /* CMF_THR_SELF */
	0x0C,  /* CMF_THR_BTN_MUT */
	0x0C,  /* CMF_THR_BTN_SELF */
	0x77, 0x77,  /* RAW_FILTER_MASK_CA */
	0x00,  /* RAW_FILT_IIR_COEFF_MUTUAL_CA */
	0x28,  /* RAW_FILT_IIR_THRESHOLD_MUTUAL_CA */
	0x00,  /* RAW_FILT_IIR_COEFF_SELF_CA */
	0x0F,  /* RAW_FILT_IIR_THRESHOLD_SELF_CA */
	0x01,  /* RAW_FILT_IIR_COEFF_BALANCED_CA */
	0x14,  /* RAW_FILT_IIR_THRESHOLD_BALANCED_CA */
	0x01,  /* RAW_FILT_IIR_COEFF_BUTTONS_CA */
	0x14,  /* RAW_FILT_IIR_THRESHOLD_BUTTONS_CA */
	0x28,  /* CMF_THR_MUT_CA */
	0x0C,  /* CMF_THR_SELF_CA */
	0x0C,  /* CMF_THR_BTN_MUT_CA */
	0x0C,  /* CMF_THR_BTN_SELF_CA */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* Reserved544 */
	0x00, 0x00, 0x00, 0x00,  /* Reserved552 */
	0x00, 0x00,  /* Reserved556 */
	0x05,  /* BL_DELAY_MUT */
	0x05,  /* BL_DELAY_SELF */
	0x14,  /* BL_DELAY_BAL */
	0x03,  /* BL_DELAY_BTN */
	0x14,  /* BL_DELAY_MUT_CA */
	0x14,  /* BL_DELAY_SELF_CA */
	0x14,  /* BL_DELAY_BTN_CA */
	0x0E,  /* BL_THR_MUT */
	0x06,  /* BL_THR_SELF */
	0x0A,  /* BL_THR_BAL */
	0x0A,  /* BL_THR_BTN_MUT */
	0x0A,  /* BL_THR_BTN_SELF */
	0x14,  /* BL_THR_MUT_CA */
	0x0A,  /* BL_THR_SELF_CA */
	0x50,  /* BL_FILT_MUT */
	0x10,  /* BL_FILT_SELF */
	0x50,  /* BL_FILT_BAL */
	0x50,  /* BL_FILT_BTN_MUT */
	0x50,  /* BL_FILT_BTN_SELF */
	0x0C,  /* CMF_THR_BAL */
	0x05,  /* CMF_DELTA_RESET_COUNTER */
	0xF2,  /* SELF_SD_RESET */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* Reserved580 */
	0x08, 0x00, 0x00, 0x00,  /* H2OREJECTION_SIZE */
	0x01,  /* BL_H20_RJCT */
	0x04,  /* BL_H20_SNS_WIDTH */
	0x00, 0x00,  /* Reserved598 */
	0xB4, 0x01, 0x00, 0x00,  /* CDC_CFG_SIZE */
	0x05, 0x04, 0xFF, 0x81,  /* TSS_CONTROL_MUT */
	0x03, 0x02, 0x00, 0x00,  /* TSS_LENGTH_MUT */
	0xFF, 0x00, 0xAF, 0xC0,  /* TSS_TX_CONFIG_MUT */
	0xFF, 0xFF, 0xB0, 0x00,  /* TSS_TX_CONTROL_MUT */
	0x22, 0x02, 0x7F, 0x81,  /* TSS_SEQ_CONFIG1_MUT */
	0x7F, 0x72, 0x01, 0x80,  /* TSS_SEQ_CONFIG2_MUT */
	0x08, 0x01, 0x48, 0x04,  /* TSS_SEQ_CONFIG3_MUT */
	0x54, 0x34, 0x3C, 0x32,  /* TSS_SEQ_CONFIG4_MUT */
	0x12, 0x01, 0x01, 0x00,  /* TSS_SEQ_CONFIG5_MUT */
	0x04, 0x01, 0x08, 0x10,  /* TSS_SEQ_CONFIG6_MUT */
	0x01, 0x04, 0x00, 0x00,  /* TSS_SEQ_CONFIG7_MUT */
	0x44, 0x01, 0x02, 0x80,  /* TSS_SEQ_CONFIG8_MUT */
	0x01, 0x01, 0x00, 0x00,  /* TSS_EXT_CONFIG1_MUT */
	0x80, 0x00, 0x01, 0x00,  /* TSS_EXT_CONFIG2_MUT */
	0x00, 0x00, 0x00, 0x00,  /* TSS_EXT_INTERVAL_MUT */
	0x01, 0x01, 0x00, 0x90,  /* TSS_INT_CONFIG1_MUT */
	0x80, 0x00, 0xFE, 0x0B,  /* TSS_INT_CONFIG2_MUT */
	0x00, 0x00, 0x00, 0x00,  /* TSS_INT_INTERVAL_MUT */
	0x00, 0x00, 0x00, 0x00,  /* TSS_MCS_CONFIG_MUT */
	0x7A, 0xA0, 0x00, 0xD0,  /* TSS_RX_CONFIG_MUT */
	0x00, 0x00, 0x00, 0x00,  /* Reserved684 */
	0x05, 0x07, 0xFF, 0x81,  /* TSS_CONTROL_SELF */
	0x02, 0x14, 0x00, 0x00,  /* TSS_LENGTH_SELF */
	0xFF, 0x06, 0xAF, 0x80,  /* TSS_TX_CONFIG_SELF */
	0xFF, 0xFF, 0xD0, 0x00,  /* TSS_TX_CONTROL_SELF */
	0x22, 0x02, 0x7F, 0x81,  /* TSS_SEQ_CONFIG1_SELF */
	0x7F, 0x72, 0x01, 0x80,  /* TSS_SEQ_CONFIG2_SELF */
	0x05, 0x01, 0x4F, 0x04,  /* TSS_SEQ_CONFIG3_SELF */
	0x48, 0x1E, 0x64, 0x1E,  /* TSS_SEQ_CONFIG4_SELF */
	0x12, 0x01, 0x01, 0x00,  /* TSS_SEQ_CONFIG5_SELF */
	0x04, 0x01, 0x08, 0x10,  /* TSS_SEQ_CONFIG6_SELF */
	0x01, 0x04, 0x00, 0x00,  /* TSS_SEQ_CONFIG7_SELF */
	0x44, 0x01, 0x02, 0x80,  /* TSS_SEQ_CONFIG8_SELF */
	0x01, 0x01, 0x00, 0x00,  /* TSS_EXT_CONFIG1_SELF */
	0x80, 0x00, 0x01, 0x00,  /* TSS_EXT_CONFIG2_SELF */
	0x00, 0x00, 0x00, 0x00,  /* TSS_EXT_INTERVAL_SELF */
	0x01, 0xE6, 0x00, 0x90,  /* TSS_INT_CONFIG1_SELF */
	0x80, 0x00, 0xCC, 0x01,  /* TSS_INT_CONFIG2_SELF */
	0x00, 0x00, 0x00, 0x00,  /* TSS_INT_INTERVAL_SELF */
	0x00, 0x00, 0x00, 0x00,  /* TSS_MCS_CONFIG_SELF */
	0x7A, 0xA6, 0x00, 0xD0,  /* TSS_RX_CONFIG_SELF */
	0x00, 0x00, 0x00, 0x00,  /* Reserved768 */
	0x05, 0x04, 0xFF, 0x80,  /* TSS_CONTROL_BAL */
	0x03, 0x20, 0x00, 0x00,  /* TSS_LENGTH_BAL */
	0xAF, 0x08, 0xAF, 0xC0,  /* TSS_TX_CONFIG_BAL */
	0xFF, 0xFF, 0xB0, 0x00,  /* TSS_TX_CONTROL_BAL */
	0x22, 0x02, 0x7F, 0x81,  /* TSS_SEQ_CONFIG1_BAL */
	0x7F, 0x72, 0x01, 0x80,  /* TSS_SEQ_CONFIG2_BAL */
	0x01, 0x81, 0x40, 0x04,  /* TSS_SEQ_CONFIG3_BAL */
	0x96, 0x0F, 0x96, 0x0F,  /* TSS_SEQ_CONFIG4_BAL */
	0x12, 0x01, 0x01, 0x00,  /* TSS_SEQ_CONFIG5_BAL */
	0x04, 0x01, 0x08, 0x10,  /* TSS_SEQ_CONFIG6_BAL */
	0x01, 0x04, 0x00, 0x00,  /* TSS_SEQ_CONFIG7_BAL */
	0x44, 0x01, 0x02, 0x80,  /* TSS_SEQ_CONFIG8_BAL */
	0x01, 0x01, 0x00, 0x00,  /* TSS_EXT_CONFIG1_BAL */
	0x80, 0x00, 0x01, 0x00,  /* TSS_EXT_CONFIG2_BAL */
	0x00, 0x00, 0x00, 0x00,  /* TSS_EXT_INTERVAL_BAL */
	0x01, 0x01, 0x00, 0x90,  /* TSS_INT_CONFIG1_BAL */
	0x80, 0x00, 0xFE, 0x0B,  /* TSS_INT_CONFIG2_BAL */
	0x00, 0x00, 0x00, 0x00,  /* TSS_INT_INTERVAL_BAL */
	0x00, 0x00, 0x00, 0x00,  /* TSS_MCS_CONFIG_BAL */
	0x3A, 0xA0, 0x00, 0xD0,  /* TSS_RX_CONFIG_BAL */
	0x05, 0x04, 0xFF, 0x80,  /* TSS_CONTROL_BTN */
	0x02, 0x04, 0x00, 0x00,  /* TSS_LENGTH_BTN_MUT */
	0xFF, 0x00, 0xAF, 0xC0,  /* TSS_TX_CONFIG_BTN */
	0xFF, 0xFF, 0xB0, 0x00,  /* TSS_TX_CONTROL_BTN */
	0x22, 0x02, 0x7F, 0x81,  /* TSS_SEQ_CONFIG1_BTN */
	0x7F, 0x72, 0x01, 0x80,  /* TSS_SEQ_CONFIG2_BTN */
	0x01, 0x81, 0x40, 0x04,  /* TSS_SEQ_CONFIG3_BTN */
	0x96, 0x0F, 0x96, 0x0F,  /* TSS_SEQ_CONFIG4_BTN */
	0x12, 0x01, 0x01, 0x00,  /* TSS_SEQ_CONFIG5_BTN */
	0x04, 0x01, 0x08, 0x10,  /* TSS_SEQ_CONFIG6_BTN */
	0x01, 0x04, 0x00, 0x00,  /* TSS_SEQ_CONFIG7_BTN */
	0x44, 0x01, 0x02, 0x80,  /* TSS_SEQ_CONFIG8_BTN */
	0x01, 0x01, 0x00, 0x00,  /* TSS_EXT_CONFIG1_BTN */
	0x80, 0x00, 0x01, 0x00,  /* TSS_EXT_CONFIG2_BTN */
	0x00, 0x00, 0x00, 0x00,  /* TSS_EXT_INTERVAL_BTN */
	0x01, 0x01, 0x00, 0x90,  /* TSS_INT_CONFIG1_BTN */
	0x80, 0x00, 0xFE, 0x0B,  /* TSS_INT_CONFIG2_BTN */
	0x00, 0x00, 0x00, 0x00,  /* TSS_INT_INTERVAL_BTN */
	0x00, 0x00, 0x00, 0x00,  /* TSS_MCS_CONFIG_BTN */
	0x3A, 0xA0, 0x00, 0xD0,  /* TSS_RX_CONFIG_BTN */
	0x08, 0x04, 0x00, 0x00,  /* TSS_LENGTH_BTN_SELF */
	0x3F, 0x00, 0x30, 0x30,  /* TSS_RX_VREF */
	0x40, 0x03, 0x07, 0x40,  /* TSS_RX_LX_CONFIG */
	0x11, 0x00, 0x00, 0x00,  /* TX_NUM */
	0x1C, 0x00, 0x00, 0x00,  /* RX_NUM */
	0x2D, 0x00, 0x00, 0x00,  /* SENS_NUM */
	0xDC, 0x01, 0x00, 0x00,  /* CROSS_NUM */
	0x00, 0x00, 0x00, 0x00,  /* BUTTON_NUM */
	0x04, 0x00, 0x00, 0x00,  /* SLOTS_MUT */
	0x04, 0x00, 0x00, 0x00,  /* SLOTS_SELF_RX */
	0x03, 0x00, 0x00, 0x00,  /* SLOTS_SELF_TX */
	0x07, 0x00, 0x00, 0x00,  /* SLOTS_SELF */
	0x0E, 0x00, 0x00, 0x00,  /* SLOTS_BAL */
	0x3C, 0x00, 0x00, 0x00,  /* SCALE_MUT */
	0xC8, 0x00, 0x00, 0x00,  /* SCALE_SELF */
	0x64, 0x00, 0x00, 0x00,  /* SCALE_BAL */
	0x64, 0x00, 0x00, 0x00,  /* SCALE_BUTTON */
	0x00, 0x00, 0x00, 0x00,  /* LX_MODE */
	0x11, 0x00, 0x00, 0x00,  /* LX_SCALE */
	0x01, 0x00, 0x00, 0x00,  /* ABSOLUTE_CR_CORRECTION_ENABLE */
	0x02, 0x00, 0x00, 0x00,  /* SCANNING_MODE_MUTUAL */
	0x02, 0x00, 0x00, 0x00,  /* SCANNING_MODE_BUTTON */
	0x0A, 0x00,  /* DETECT_CHARGER_THRESHOLD */
	0x01,  /* CA_LX_SCAN_MODE */
	0x00,  /* SUB_SLOT_SCAN */
	0x0C,  /* NOISE_METRIC1_THRESHOLD */
	0x0C,  /* NOISE_METRIC2_THRESHOLD */
	0x32,  /* NOISE_METRIC3_THRESHOLD */
	0x00,  /* AFH_DYNAMIC_THRSH_ENABLE */
	0x04,  /* ADC_CONFIG */
	0x00,  /* Reserved1029 */
	0x04,  /* TSS_LDO_PROG */
	0x02,  /* TX_PERIOD_DUMMY_SCAN */
	0x00,  /* SINGLE_ENDED_LISTEN_SCAN */
	0x01,  /* SPREAD_MTX */
	0x00,  /* SUBCONV_FILTER */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00,  /* Reserved1035 */
	0x34, 0x00, 0x00, 0x00,  /* CALIBRATION_PARAM_SIZE */
	0x01, 0x00, 0x00, 0x00,  /* Reserved1112 */
	0x01, 0x00, 0x00, 0x00,  /* Reserved1116 */
	0x01, 0x00, 0x00, 0x00,  /* Reserved1120 */
	0x01, 0x00, 0x00, 0x00,  /* Reserved1124 */
	0x00, 0x00, 0x00, 0x00,  /* GLOBAL_IDAC_LSB_MUTUAL */
	0x01, 0x00, 0x00, 0x00,  /* GLOBAL_IDAC_LSB_SELF */
	0x01, 0x00, 0x00, 0x00,  /* GLOBAL_IDAC_LSB_BALANCED */
	0x01, 0x00, 0x00, 0x00,  /* GLOBAL_IDAC_LSB_BUTTON */
	0xF6, 0xFF, 0xFF, 0xFF,  /* TARGET_LEVEL_MUTUAL */
	0xF6, 0xFF, 0xFF, 0xFF,  /* TARGET_LEVEL_SELF */
	0x00, 0x00, 0x00, 0x00,  /* TARGET_LEVEL_BALANCED */
	0x00, 0x00, 0x00, 0x00,  /* TARGET_LEVEL_BUTTON */
	0x02, 0x00, 0x00, 0x00,  /* GAIN_MUTUAL */
	0x01, 0x00, 0x00, 0x00,  /* GAIN_SELF */
	0x01, 0x00, 0x00, 0x00,  /* GAIN_BALANCED */
	0x01, 0x00, 0x00, 0x00,  /* GAIN_BTN_MUTUAL */
	0x03, 0x00, 0x00, 0x00,  /* GAIN_BTN_SELF */
	0x00, 0x00, 0x00, 0x00,  /* Reserved1180 */
	0x08, 0x00, 0x00, 0x00,  /* SPREADER_CFG_SIZE */
	0x00, 0x00, 0x00, 0x00,  /* CLK_IMO_SPREAD */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* Reserved1192 */
	0x23, 0x0C, 0x0A, 0x0D, 
	0x0B, 0x08, 0x0E, 0x0F, 
	0x09, 0x07, 0x06, 0x24, 
	0x22, 0x39, 0x2F, 0x38, 
	0x37, 0x05, 0x2D, 0x2C, 
	0x2E, 0x30, 0x31, 0x32, 
	0x33, 0x35, 0x34, 0x36, 
	0x29, 0x2B, 0x28, 0x2A, 
	0x25, 0x26, 0x18, 0x19, 
	0x15, 0x16, 0x13, 0x11, 
	0x12, 0x10, 0x27, 0x14, 
	0x17, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00,  /* CDC_PIN_INDEX_TABLE */
	0x00, 0x00, 0x00,  /* Reserved1281 */
	0x29, 0x25, 0x29, 0x25, 
	0x29, 0x25, 0x29, 0x25, 
	0x29, 0x29, 0x17, 0x17, 
	0x17, 0x17,  /* CDC_BALANCED_LX_TABLE */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00,  /* Reserved1298 */
	0x00, 0xC0, 0xFF, 0x0F, 
	0xE0, 0x1F, 0x00, 0x00, 
	0x00, 0x7F, 0x00, 0xE0, 
	0xFF, 0x00, 0x1E, 0x00, 
	0x00, 0x00,  /* CDC_BALANCED_TX_PATTERNS */
	0x00, 0x00,  /* Reserved1322 */
	0xFD, 0x00, 0x07, 0x17, 
	0x05, 0x00, 0x02, 0x04, 
	0x01, 0x03, 0x06, 0x07, 
	0x00, 0x0B, 0x0F, 0x0B, 
	0x0B, 0x0A, 0x0A, 0x09, 
	0x0A, 0x0F, 0xFA, 0x00, 
	0x06, 0x17, 0x00, 0x08, 
	0x00, 0x00, 0x1A, 0x19, 
	0x1B, 0x09, 0x00, 0x0F, 
	0x0B, 0x0F, 0x0A, 0x0A, 
	0x0A, 0x0A, 0x0A, 0x0F, 
	0xF7, 0x00, 0x07, 0x17, 
	0x0F, 0x0D, 0x0C, 0x00, 
	0x0B, 0x11, 0x0A, 0x0E, 
	0x00, 0x0A, 0x0B, 0x0B, 
	0x0F, 0x0B, 0x0A, 0x0A, 
	0x0A, 0x0F, 0xFF, 0x00, 
	0x08, 0x17, 0x15, 0x16, 
	0x17, 0x18, 0x13, 0x12, 
	0x14, 0x10, 0x00, 0x0B, 
	0x0A, 0x0B, 0x0B, 0x0B, 
	0x0B, 0x0A, 0x0A, 0x0F, 
	0xEF, 0x00, 0x07, 0x29, 
	0x29, 0x27, 0x28, 0x26, 
	0x00, 0x24, 0x25, 0x2A, 
	0x00, 0x0B, 0x0B, 0x0A, 
	0x09, 0x0F, 0x0A, 0x0A, 
	0x0B, 0x0F, 0x6F, 0x00, 
	0x06, 0x17, 0x1E, 0x1C, 
	0x1F, 0x1D, 0x00, 0x20, 
	0x21, 0x00, 0x00, 0x0B, 
	0x0B, 0x0B, 0x0B, 0x0F, 
	0x0B, 0x0B, 0x0F, 0x0F, 
	0x93, 0x00, 0x04, 0x29, 
	0x22, 0x23, 0x00, 0x00, 
	0x2B, 0x00, 0x00, 0x2C, 
	0x00, 0x0B, 0x0B, 0x0F, 
	0x0F, 0x09, 0x0F, 0x0F, 
	0x0A, 0x0F, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* CDC_SLOT_TABLE */
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00,  /* Reserved1500 */
	0xA3, 0x8D,  /* CONFIG_CRC */
};

/* Touchscreen Parameters Field Sizes (Writable: 0:Readonly; 1:Writable) */
static const uint16_t cyttsp4_param_size[] = {
/*	Size	Name	*/
	2, /* CONFIG_DATA_SIZE */
	2, /* CONFIG_DATA_MAX_SIZE */
	4, /* SDK_CTRL_CFG_SIZE */
	2, /* CONFIG_VER */
	1, /* PANELID_ENABLE */
	1, /* IMO_FREQ_MHZ */
	2, /* X_LEN_PHY */
	2, /* Y_LEN_PHY */
	1, /* HST_MODE0 */
	1, /* ACT_DIST0 */
	1, /* SCAN_TYP0 */
	1, /* ACT_INTRVL0 */
	1, /* ACT_LFT_INTRVL0 */
	1, /* Reserved21 */
	2, /* LP_INTRVL0 */
	2, /* TCH_TMOUT0 */
	1, /* PWR_CFG */
	1, /* Reserved27 */
	1, /* INT_PULSE_DATA */
	1, /* OPMODE_CFG */
	2, /* HANDSHAKE_TIMEOUT */
	2, /* ESD_COUNTER_CFG */
	1, /* TIMER_CAL_INTERVAL */
	1, /* Reserved35 */
	2, /* RP2P_MIN */
	2, /* ILEAK_MAX */
	2, /* RFB_P2P */
	2, /* RFB_EXT */
	1, /* IDACOPEN_LOW */
	1, /* IDACOPEN_HIGH */
	1, /* IDACOPEN_BUTTON_LOW */
	1, /* IDACOPEN_BUTTON_HIGH */
	1, /* GIDAC_OPEN */
	1, /* GAIN_OPEN */
	1, /* GIDAC_BUTTON_OPEN */
	1, /* GAIN_BUTTON_OPEN */
	1, /* POST_CFG */
	1, /* GESTURE_CFG */
	1, /* GEST_EN0 */
	1, /* GEST_EN1 */
	1, /* GEST_EN2 */
	1, /* GEST_EN3 */
	1, /* GEST_EN4 */
	1, /* GEST_EN5 */
	1, /* GEST_EN6 */
	1, /* GEST_EN7 */
	1, /* GEST_EN8 */
	1, /* GEST_EN9 */
	1, /* GEST_EN10 */
	1, /* GEST_EN11 */
	1, /* GEST_EN12 */
	1, /* GEST_EN13 */
	1, /* GEST_EN14 */
	1, /* GEST_EN15 */
	1, /* GEST_EN16 */
	1, /* GEST_EN17 */
	1, /* GEST_EN18 */
	1, /* GEST_EN19 */
	1, /* GEST_EN20 */
	1, /* GEST_EN21 */
	1, /* GEST_EN22 */
	1, /* GEST_EN23 */
	1, /* GEST_EN24 */
	1, /* GEST_EN25 */
	1, /* GEST_EN26 */
	1, /* GEST_EN27 */
	1, /* GEST_EN28 */
	1, /* GEST_EN29 */
	1, /* GEST_EN30 */
	1, /* GEST_EN31 */
	1, /* ACT_DIST2 */
	1, /* ACT_DIST_TOUCHDOWN */
	1, /* ACT_DIST_LIFTOFF */
	1, /* ACT_DIST_COUNTER */
	1, /* ACT_DIST_Z_THRESHOLD */
	1, /* EXTERN_SYNC */
	4, /* Reserved92 */
	4, /* GRIP_CFG_SIZE */
	2, /* GRIP_XEDG_A */
	2, /* GRIP_XEDG_B */
	2, /* GRIP_XEXC_A */
	2, /* GRIP_XEXC_B */
	2, /* GRIP_YEDG_A */
	2, /* GRIP_YEDG_B */
	2, /* GRIP_YEXC_A */
	2, /* GRIP_YEXC_B */
	1, /* GRIP_FIRST_EXC */
	1, /* GRIP_EXC_EDGE_ORIGIN */
	2, /* Reserved118 */
	4, /* TRUETOUCH_CFG_SIZE */
	4, /* MAX_SELF_SCAN_INTERVAL */
	4, /* MAX_MUTUAL_SCAN_INTERVAL */
	4, /* MAX_BALANCED_SCAN_INTERVAL */
	4, /* SELF_Z_THRSH */
	4, /* SELF_Z_MODE */
	4, /* SMART_SCAN_ENABLE */
	4, /* T_COMP_ENABLE */
	4, /* T_COMP_INTERVAL */
	4, /* T_COMP_RECAL_MUTUAL_SENSOR_LIMIT */
	4, /* T_COMP_RECAL_MUTUAL_HIGH */
	4, /* T_COMP_RECAL_MUTUAL_LOW */
	4, /* T_COMP_RECAL_SELF_SENSOR_LIMIT */
	4, /* T_COMP_RECAL_SELF_HIGH */
	4, /* T_COMP_RECAL_SELF_LOW */
	4, /* CHARGER_ARMOR_ENABLE */
	2, /* AFH_ENABLE */
	2, /* AFH_SELF_ENABLE */
	4, /* AFH_LISTENING_SCAN_COUNT */
	4, /* AFH_LISTEN_SCAN_CYCLE_REPEATS */
	4, /* AFH_LISTEN_SCAN_CONFIG */
	4, /* CA_BLOCK_NOISE_THRESHOLD */
	4, /* CA_BLOCK_NOISE_HYSTERESIS */
	4, /* CA_DEFAULT_REVERT_TIME */
	2, /* CA_SMART_H2O_REJECT */
	2, /* CA_HOST_CONTROLLED_CHARGER */
	2, /* T_COMP_BUTTON_MUTUAL_HIGH */
	2, /* T_COMP_BUTTON_MUTUAL_LOW */
	2, /* T_COMP_BUTTON_SELF_HIGH */
	2, /* T_COMP_BUTTON_SELF_LOW */
	4, /* CA_NUM_SUB_CONV_BASE_SELF */
	4, /* CA_ALT_NUM_SUB_CONV_SELF */
	1, /* BTN_SCAN_CFG */
	7, /* Reserved233 */
	1, /* CA_ALT_NUM_SUB_CONV_MUTUAL */
	1, /* CA_ALT_ACQUISITION_FLAGS */
	1, /* AFH_ALT1_TX_PERIOD */
	1, /* AFH_ALT1_SCALE_MUT */
	1, /* AFH_ALT1_SCALE_SELF */
	1, /* AFH_ALT1_TX_VOLTAGE */
	1, /* AFH_ALT1_TX_PULSES */
	1, /* AFH_ALT2_TX_PERIOD */
	1, /* AFH_ALT2_SCALE_MUT */
	1, /* AFH_ALT2_SCALE_SELF */
	1, /* AFH_ALT2_TX_VOLTAGE */
	1, /* AFH_ALT2_TX_PULSES */
	1, /* GEST_CFG_SIZE */
	1, /* PAN_ACT_DSTX */
	1, /* PAN_ACT_DSTY */
	1, /* ZOOM_ACT_DSTX */
	1, /* ZOOM_ACT_DSTY */
	1, /* FLICK_ACT_DISTX */
	1, /* FLICK_ACT_DISTY */
	1, /* FLICK_TIME */
	1, /* ST_DEBOUNCE */
	1, /* MT_DEBOUNCE_PAN */
	1, /* MT_DEBOUNCE_ZOOM */
	1, /* MT_DEBOUNCE_P2Z */
	1, /* ROT_DEBOUNCE */
	1, /* COMPL_DEBOUNCE */
	2, /* MT_TIMEOUT */
	1, /* ST_DBLCLK_RMAX */
	1, /* ST_CLICK_DISTX */
	1, /* ST_CLICK_DISTY */
	1, /* Reserved271 */
	2, /* MT_CLICK_TMAX */
	2, /* MT_CLICK_TMIN */
	2, /* ST_CLICK_TMAX */
	2, /* ST_CLICK_TMIN */
	2, /* ST_DBLCLK_TMAX */
	2, /* ST_DBLCLK_TMIN */
	1, /* GESTURE_GROUP_MASK */
	1, /* GESTURE_GROUP1_START */
	1, /* GESTURE_GROUP1_END */
	1, /* GESTURE_GROUP2_START */
	1, /* GESTURE_GROUP2_END */
	1, /* GESTURE_GROUP3_START */
	1, /* GESTURE_GROUP3_END */
	1, /* GESTURE_GROUP4_START */
	1, /* GESTURE_GROUP4_END */
	3, /* Reserved293 */
	4, /* XY_FILT_CFG_SIZE */
	4, /* XY_FILTER_MASK */
	4, /* XY_FILT_IIR_COEFF */
	4, /* XY_FILT_Z_IIR_COEFF */
	1, /* XY_FILT_XY_FAST_THR */
	1, /* XY_FILT_XY_SLOW_THR */
	1, /* XY_FILT_IIR_FAST_COEFF */
	1, /* Reserved315 */
	4, /* XY_FILTER_MASK_CA */
	4, /* XY_FILT_IIR_COEFF_CA */
	4, /* XY_FILT_Z_IIR_COEFF_CA */
	1, /* XY_FILT_XY_FAST_THR_CA */
	1, /* XY_FILT_XY_SLOW_THR_CA */
	1, /* XY_FILT_IIR_FAST_COEFF_CA */
	1, /* Reserved331 */
	1, /* XY_FILT_ADAPTIVE_IIR_FILTER */
	1, /* XY_FILT_ADAPTIVE_IIR_FILTER_DISTANCE */
	1, /* XY_FILT_TOUCH_SIZE_IIR_COEFF */
	1, /* XY_FILT_TOUCH_SIZE_HYST */
	1, /* XY_FILT_TOUCH_ORIENTATION_IIR_COEFF */
	1, /* XY_FILT_TOUCH_ORIENTATION_HYST */
	1, /* XY_FILT_TOUCH_SCALLOPING_ENABLE */
	9, /* Reserved339 */
	4, /* FINGER_ID_CFG_SIZE */
	4, /* Reserved352 */
	2, /* MAX_FINGER_VELOCITY */
	2, /* MAX_FINGER_VELOCITY_CA */
	1, /* LIFTOFF_DEBOUNCE */
	1, /* LIFTOFF_DEBOUNCE_CA */
	6, /* Reserved362 */
	4, /* CENTROID_SH_CFG_SIZE */
	4, /* STYLUS_THRSH */
	4, /* STYLUS_HYST */
	4, /* S2F_THRESHOLD */
	4, /* HOVER_THRSH */
	4, /* HOVER_HYST */
	4, /* Reserved392 */
	4, /* ID_COORDS_CFG_SIZE */
	1, /* LRG_OBJ_CFG */
	1, /* FINGER_THRESH_MUTUAL */
	1, /* FINGER_THR_MUT_HYST */
	1, /* FINGER_THRESH_SELF */
	1, /* INNER_EDGE_GAIN */
	1, /* OUTER_EDGE_GAIN */
	2, /* X_RESOLUTION */
	2, /* Y_RESOLUTION */
	1, /* SENSOR_ASSIGNMENT */
	1, /* Z_SCALING */
	1, /* RX_LINE_FILTER */
	1, /* RX_LINE_FILTER_THRESHOLD */
	1, /* RX_LINE_FILTER_DEBOUNCE */
	1, /* BYPASS_THRESHOLD_GAIN */
	1, /* BYPASS_THRESHOLD_EDGE_GAIN */
	1, /* MAX_FAT_FINGER_SIZE */
	1, /* MIN_FAT_FINGER_SIZE */
	1, /* MAX_FAT_FINGER_SIZE_HYST */
	1, /* MIN_FAT_FINGER_SIZE_HYST */
	1, /* FAT_FINGER_COEFF */
	1, /* FAT_FINGER_SD_THRESHOLD */
	1, /* FAT_FINGER_COEFF_SD */
	1, /* MULTI_TOUCH_DEBOUNCE */
	1, /* MULTI_TOUCH_DEBOUNCE_CA */
	1, /* FINGER_THRESHOLD_MUTUAL_CA */
	1, /* FINGER_THRESH_SELF_CA */
	1, /* SIZE_ORIENTATION_ENABLE */
	1, /* MAJOR_AXIS_OFFSET */
	1, /* MAJOR_AXIS_SCALE */
	1, /* MINOR_AXIS_OFFSET */
	1, /* MINOR_AXIS_SCALE */
	1, /* CLIPPING_X_LOW */
	1, /* CLIPPING_X_HIGH */
	1, /* CLIPPING_Y_LOW */
	1, /* CLIPPING_Y_HIGH */
	1, /* CLIPPING_BOUNDARY_REMOVE */
	1, /* Reserved438 */
	1, /* COEF_EDGE_ATTRACTION */
	2, /* WIDTH_CORNER_DIAG1 */
	2, /* WIDTH_CORNER_DIAG2 */
	2, /* WIDTH_CORNER_DIAG3 */
	2, /* WIDTH_CORNER_DIAG4 */
	2, /* WIDTH_CORNER_DIAG5 */
	2, /* WIDTH_CORNER_DIAG6 */
	2, /* WIDTH_CORNER_DIAG7 */
	2, /* WIDTH_CORNER_DIAG8 */
	2, /* WIDTH_CORNER_DIAG9 */
	2, /* WIDTH_CORNER_DIAG10 */
	2, /* WIDTH_CORNER_DIAG11 */
	2, /* WIDTH_CORNER_PERP1 */
	2, /* WIDTH_CORNER_PERP2 */
	2, /* WIDTH_CORNER_PERP3 */
	2, /* WIDTH_CORNER_PERP4 */
	1, /* NOISE_REJECTION_3x3_FILTER_SCALE */
	1, /* NOISE_REJECTION_3x3_FILTER_SCALE_CA */
	1, /* DIRECT_XY_ENABLE */
	1, /* CALC_THRESHOLD */
	2, /* JIG_MODE_CONFIG */
	8, /* Reserved476 */
	4, /* BTN_CFG_SIZE */
	2, /* BTN_THRSH_MUT_0 */
	2, /* BTN_THRSH_MUT_1 */
	2, /* BTN_THRSH_MUT_2 */
	2, /* BTN_THRSH_MUT_3 */
	1, /* BTN_HYST_MUT */
	1, /* Reserved497 */
	1, /* Reserved498 */
	1, /* Reserved499 */
	2, /* BTN_THRSH_SELF */
	2, /* BTN_THRSH_SELF_1 */
	2, /* BTN_THRSH_SELF_2 */
	2, /* BTN_THRSH_SELF_3 */
	1, /* BTN_HYST_SELF */
	1, /* Reserved509 */
	1, /* Reserved510 */
	1, /* Reserved511 */
	4, /* RAW_PROC_CFG_SIZE */
	2, /* RAW_FILTER_MASK */
	1, /* RAW_FILT_IIR_COEFF_MUTUAL */
	1, /* RAW_FILT_IIR_THRESHOLD_MUTUAL */
	1, /* RAW_FILT_IIR_COEFF_SELF */
	1, /* RAW_FILT_IIR_THRESHOLD_SELF */
	1, /* RAW_FILT_IIR_COEFF_BALANCED */
	1, /* RAW_FILT_IIR_THRESHOLD_BALANCED */
	1, /* RAW_FILT_IIR_COEFF_BUTTONS */
	1, /* RAW_FILT_IIR_THRESHOLD_BUTTONS */
	1, /* CMF_THR_MUT */
	1, /* CMF_THR_SELF */
	1, /* CMF_THR_BTN_MUT */
	1, /* CMF_THR_BTN_SELF */
	2, /* RAW_FILTER_MASK_CA */
	1, /* RAW_FILT_IIR_COEFF_MUTUAL_CA */
	1, /* RAW_FILT_IIR_THRESHOLD_MUTUAL_CA */
	1, /* RAW_FILT_IIR_COEFF_SELF_CA */
	1, /* RAW_FILT_IIR_THRESHOLD_SELF_CA */
	1, /* RAW_FILT_IIR_COEFF_BALANCED_CA */
	1, /* RAW_FILT_IIR_THRESHOLD_BALANCED_CA */
	1, /* RAW_FILT_IIR_COEFF_BUTTONS_CA */
	1, /* RAW_FILT_IIR_THRESHOLD_BUTTONS_CA */
	1, /* CMF_THR_MUT_CA */
	1, /* CMF_THR_SELF_CA */
	1, /* CMF_THR_BTN_MUT_CA */
	1, /* CMF_THR_BTN_SELF_CA */
	8, /* Reserved544 */
	4, /* Reserved552 */
	2, /* Reserved556 */
	1, /* BL_DELAY_MUT */
	1, /* BL_DELAY_SELF */
	1, /* BL_DELAY_BAL */
	1, /* BL_DELAY_BTN */
	1, /* BL_DELAY_MUT_CA */
	1, /* BL_DELAY_SELF_CA */
	1, /* BL_DELAY_BTN_CA */
	1, /* BL_THR_MUT */
	1, /* BL_THR_SELF */
	1, /* BL_THR_BAL */
	1, /* BL_THR_BTN_MUT */
	1, /* BL_THR_BTN_SELF */
	1, /* BL_THR_MUT_CA */
	1, /* BL_THR_SELF_CA */
	1, /* BL_FILT_MUT */
	1, /* BL_FILT_SELF */
	1, /* BL_FILT_BAL */
	1, /* BL_FILT_BTN_MUT */
	1, /* BL_FILT_BTN_SELF */
	1, /* CMF_THR_BAL */
	1, /* CMF_DELTA_RESET_COUNTER */
	1, /* SELF_SD_RESET */
	12, /* Reserved580 */
	4, /* H2OREJECTION_SIZE */
	1, /* BL_H20_RJCT */
	1, /* BL_H20_SNS_WIDTH */
	2, /* Reserved598 */
	4, /* CDC_CFG_SIZE */
	4, /* TSS_CONTROL_MUT */
	4, /* TSS_LENGTH_MUT */
	4, /* TSS_TX_CONFIG_MUT */
	4, /* TSS_TX_CONTROL_MUT */
	4, /* TSS_SEQ_CONFIG1_MUT */
	4, /* TSS_SEQ_CONFIG2_MUT */
	4, /* TSS_SEQ_CONFIG3_MUT */
	4, /* TSS_SEQ_CONFIG4_MUT */
	4, /* TSS_SEQ_CONFIG5_MUT */
	4, /* TSS_SEQ_CONFIG6_MUT */
	4, /* TSS_SEQ_CONFIG7_MUT */
	4, /* TSS_SEQ_CONFIG8_MUT */
	4, /* TSS_EXT_CONFIG1_MUT */
	4, /* TSS_EXT_CONFIG2_MUT */
	4, /* TSS_EXT_INTERVAL_MUT */
	4, /* TSS_INT_CONFIG1_MUT */
	4, /* TSS_INT_CONFIG2_MUT */
	4, /* TSS_INT_INTERVAL_MUT */
	4, /* TSS_MCS_CONFIG_MUT */
	4, /* TSS_RX_CONFIG_MUT */
	4, /* Reserved684 */
	4, /* TSS_CONTROL_SELF */
	4, /* TSS_LENGTH_SELF */
	4, /* TSS_TX_CONFIG_SELF */
	4, /* TSS_TX_CONTROL_SELF */
	4, /* TSS_SEQ_CONFIG1_SELF */
	4, /* TSS_SEQ_CONFIG2_SELF */
	4, /* TSS_SEQ_CONFIG3_SELF */
	4, /* TSS_SEQ_CONFIG4_SELF */
	4, /* TSS_SEQ_CONFIG5_SELF */
	4, /* TSS_SEQ_CONFIG6_SELF */
	4, /* TSS_SEQ_CONFIG7_SELF */
	4, /* TSS_SEQ_CONFIG8_SELF */
	4, /* TSS_EXT_CONFIG1_SELF */
	4, /* TSS_EXT_CONFIG2_SELF */
	4, /* TSS_EXT_INTERVAL_SELF */
	4, /* TSS_INT_CONFIG1_SELF */
	4, /* TSS_INT_CONFIG2_SELF */
	4, /* TSS_INT_INTERVAL_SELF */
	4, /* TSS_MCS_CONFIG_SELF */
	4, /* TSS_RX_CONFIG_SELF */
	4, /* Reserved768 */
	4, /* TSS_CONTROL_BAL */
	4, /* TSS_LENGTH_BAL */
	4, /* TSS_TX_CONFIG_BAL */
	4, /* TSS_TX_CONTROL_BAL */
	4, /* TSS_SEQ_CONFIG1_BAL */
	4, /* TSS_SEQ_CONFIG2_BAL */
	4, /* TSS_SEQ_CONFIG3_BAL */
	4, /* TSS_SEQ_CONFIG4_BAL */
	4, /* TSS_SEQ_CONFIG5_BAL */
	4, /* TSS_SEQ_CONFIG6_BAL */
	4, /* TSS_SEQ_CONFIG7_BAL */
	4, /* TSS_SEQ_CONFIG8_BAL */
	4, /* TSS_EXT_CONFIG1_BAL */
	4, /* TSS_EXT_CONFIG2_BAL */
	4, /* TSS_EXT_INTERVAL_BAL */
	4, /* TSS_INT_CONFIG1_BAL */
	4, /* TSS_INT_CONFIG2_BAL */
	4, /* TSS_INT_INTERVAL_BAL */
	4, /* TSS_MCS_CONFIG_BAL */
	4, /* TSS_RX_CONFIG_BAL */
	4, /* TSS_CONTROL_BTN */
	4, /* TSS_LENGTH_BTN_MUT */
	4, /* TSS_TX_CONFIG_BTN */
	4, /* TSS_TX_CONTROL_BTN */
	4, /* TSS_SEQ_CONFIG1_BTN */
	4, /* TSS_SEQ_CONFIG2_BTN */
	4, /* TSS_SEQ_CONFIG3_BTN */
	4, /* TSS_SEQ_CONFIG4_BTN */
	4, /* TSS_SEQ_CONFIG5_BTN */
	4, /* TSS_SEQ_CONFIG6_BTN */
	4, /* TSS_SEQ_CONFIG7_BTN */
	4, /* TSS_SEQ_CONFIG8_BTN */
	4, /* TSS_EXT_CONFIG1_BTN */
	4, /* TSS_EXT_CONFIG2_BTN */
	4, /* TSS_EXT_INTERVAL_BTN */
	4, /* TSS_INT_CONFIG1_BTN */
	4, /* TSS_INT_CONFIG2_BTN */
	4, /* TSS_INT_INTERVAL_BTN */
	4, /* TSS_MCS_CONFIG_BTN */
	4, /* TSS_RX_CONFIG_BTN */
	4, /* TSS_LENGTH_BTN_SELF */
	4, /* TSS_RX_VREF */
	4, /* TSS_RX_LX_CONFIG */
	4, /* TX_NUM */
	4, /* RX_NUM */
	4, /* SENS_NUM */
	4, /* CROSS_NUM */
	4, /* BUTTON_NUM */
	4, /* SLOTS_MUT */
	4, /* SLOTS_SELF_RX */
	4, /* SLOTS_SELF_TX */
	4, /* SLOTS_SELF */
	4, /* SLOTS_BAL */
	4, /* SCALE_MUT */
	4, /* SCALE_SELF */
	4, /* SCALE_BAL */
	4, /* SCALE_BUTTON */
	4, /* LX_MODE */
	4, /* LX_SCALE */
	4, /* ABSOLUTE_CR_CORRECTION_ENABLE */
	4, /* SCANNING_MODE_MUTUAL */
	4, /* SCANNING_MODE_BUTTON */
	2, /* DETECT_CHARGER_THRESHOLD */
	1, /* CA_LX_SCAN_MODE */
	1, /* SUB_SLOT_SCAN */
	1, /* NOISE_METRIC1_THRESHOLD */
	1, /* NOISE_METRIC2_THRESHOLD */
	1, /* NOISE_METRIC3_THRESHOLD */
	1, /* AFH_DYNAMIC_THRSH_ENABLE */
	1, /* ADC_CONFIG */
	1, /* Reserved1029 */
	1, /* TSS_LDO_PROG */
	1, /* TX_PERIOD_DUMMY_SCAN */
	1, /* SINGLE_ENDED_LISTEN_SCAN */
	1, /* SPREAD_MTX */
	1, /* SUBCONV_FILTER */
	73, /* Reserved1035 */
	4, /* CALIBRATION_PARAM_SIZE */
	4, /* Reserved1112 */
	4, /* Reserved1116 */
	4, /* Reserved1120 */
	4, /* Reserved1124 */
	4, /* GLOBAL_IDAC_LSB_MUTUAL */
	4, /* GLOBAL_IDAC_LSB_SELF */
	4, /* GLOBAL_IDAC_LSB_BALANCED */
	4, /* GLOBAL_IDAC_LSB_BUTTON */
	4, /* TARGET_LEVEL_MUTUAL */
	4, /* TARGET_LEVEL_SELF */
	4, /* TARGET_LEVEL_BALANCED */
	4, /* TARGET_LEVEL_BUTTON */
	4, /* GAIN_MUTUAL */
	4, /* GAIN_SELF */
	4, /* GAIN_BALANCED */
	4, /* GAIN_BTN_MUTUAL */
	4, /* GAIN_BTN_SELF */
	4, /* Reserved1180 */
	4, /* SPREADER_CFG_SIZE */
	4, /* CLK_IMO_SPREAD */
	24, /* Reserved1192 */
	65, /* CDC_PIN_INDEX_TABLE */
	3, /* Reserved1281 */
	14, /* CDC_BALANCED_LX_TABLE */
	6, /* Reserved1298 */
	18, /* CDC_BALANCED_TX_PATTERNS */
	2, /* Reserved1322 */
	176, /* CDC_SLOT_TABLE */
	32, /* Reserved1500 */
	2, /* CONFIG_CRC */
};

/* Touchscreen Parameters Field Address*/
static const uint8_t cyttsp4_param_addr[] = {
/*	Address	Name	*/
	0xDC, 0x00, /* CONFIG_DATA_SIZE */
	0xDC, 0x02, /* CONFIG_DATA_MAX_SIZE */
	0xDC, 0x04, /* SDK_CTRL_CFG_SIZE */
	0xDC, 0x08, /* CONFIG_VER */
	0xDC, 0x0A, /* PANELID_ENABLE */
	0xDC, 0x0B, /* IMO_FREQ_MHZ */
	0xDC, 0x0C, /* X_LEN_PHY */
	0xDC, 0x0E, /* Y_LEN_PHY */
	0xDC, 0x10, /* HST_MODE0 */
	0xDC, 0x11, /* ACT_DIST0 */
	0xDC, 0x12, /* SCAN_TYP0 */
	0xDC, 0x13, /* ACT_INTRVL0 */
	0xDC, 0x14, /* ACT_LFT_INTRVL0 */
	0xDC, 0x15, /* Reserved21 */
	0xDC, 0x16, /* LP_INTRVL0 */
	0xDC, 0x18, /* TCH_TMOUT0 */
	0xDC, 0x1A, /* PWR_CFG */
	0xDC, 0x1B, /* Reserved27 */
	0xDC, 0x1C, /* INT_PULSE_DATA */
	0xDC, 0x1D, /* OPMODE_CFG */
	0xDC, 0x1E, /* HANDSHAKE_TIMEOUT */
	0xDC, 0x20, /* ESD_COUNTER_CFG */
	0xDC, 0x22, /* TIMER_CAL_INTERVAL */
	0xDC, 0x23, /* Reserved35 */
	0xDC, 0x24, /* RP2P_MIN */
	0xDC, 0x26, /* ILEAK_MAX */
	0xDC, 0x28, /* RFB_P2P */
	0xDC, 0x2A, /* RFB_EXT */
	0xDC, 0x2C, /* IDACOPEN_LOW */
	0xDC, 0x2D, /* IDACOPEN_HIGH */
	0xDC, 0x2E, /* IDACOPEN_BUTTON_LOW */
	0xDC, 0x2F, /* IDACOPEN_BUTTON_HIGH */
	0xDC, 0x30, /* GIDAC_OPEN */
	0xDC, 0x31, /* GAIN_OPEN */
	0xDC, 0x32, /* GIDAC_BUTTON_OPEN */
	0xDC, 0x33, /* GAIN_BUTTON_OPEN */
	0xDC, 0x34, /* POST_CFG */
	0xDC, 0x35, /* GESTURE_CFG */
	0xDC, 0x36, /* GEST_EN0 */
	0xDC, 0x37, /* GEST_EN1 */
	0xDC, 0x38, /* GEST_EN2 */
	0xDC, 0x39, /* GEST_EN3 */
	0xDC, 0x3A, /* GEST_EN4 */
	0xDC, 0x3B, /* GEST_EN5 */
	0xDC, 0x3C, /* GEST_EN6 */
	0xDC, 0x3D, /* GEST_EN7 */
	0xDC, 0x3E, /* GEST_EN8 */
	0xDC, 0x3F, /* GEST_EN9 */
	0xDC, 0x40, /* GEST_EN10 */
	0xDC, 0x41, /* GEST_EN11 */
	0xDC, 0x42, /* GEST_EN12 */
	0xDC, 0x43, /* GEST_EN13 */
	0xDC, 0x44, /* GEST_EN14 */
	0xDC, 0x45, /* GEST_EN15 */
	0xDC, 0x46, /* GEST_EN16 */
	0xDC, 0x47, /* GEST_EN17 */
	0xDC, 0x48, /* GEST_EN18 */
	0xDC, 0x49, /* GEST_EN19 */
	0xDC, 0x4A, /* GEST_EN20 */
	0xDC, 0x4B, /* GEST_EN21 */
	0xDC, 0x4C, /* GEST_EN22 */
	0xDC, 0x4D, /* GEST_EN23 */
	0xDC, 0x4E, /* GEST_EN24 */
	0xDC, 0x4F, /* GEST_EN25 */
	0xDC, 0x50, /* GEST_EN26 */
	0xDC, 0x51, /* GEST_EN27 */
	0xDC, 0x52, /* GEST_EN28 */
	0xDC, 0x53, /* GEST_EN29 */
	0xDC, 0x54, /* GEST_EN30 */
	0xDC, 0x55, /* GEST_EN31 */
	0xDC, 0x56, /* ACT_DIST2 */
	0xDC, 0x57, /* ACT_DIST_TOUCHDOWN */
	0xDC, 0x58, /* ACT_DIST_LIFTOFF */
	0xDC, 0x59, /* ACT_DIST_COUNTER */
	0xDC, 0x5A, /* ACT_DIST_Z_THRESHOLD */
	0xDC, 0x5B, /* EXTERN_SYNC */
	0xDC, 0x5C, /* Reserved92 */
	0xDC, 0x60, /* GRIP_CFG_SIZE */
	0xDC, 0x64, /* GRIP_XEDG_A */
	0xDC, 0x66, /* GRIP_XEDG_B */
	0xDC, 0x68, /* GRIP_XEXC_A */
	0xDC, 0x6A, /* GRIP_XEXC_B */
	0xDC, 0x6C, /* GRIP_YEDG_A */
	0xDC, 0x6E, /* GRIP_YEDG_B */
	0xDC, 0x70, /* GRIP_YEXC_A */
	0xDC, 0x72, /* GRIP_YEXC_B */
	0xDC, 0x74, /* GRIP_FIRST_EXC */
	0xDC, 0x75, /* GRIP_EXC_EDGE_ORIGIN */
	0xDC, 0x76, /* Reserved118 */
	0xDC, 0x78, /* TRUETOUCH_CFG_SIZE */
	0xDC, 0x7C, /* MAX_SELF_SCAN_INTERVAL */
	0xDC, 0x80, /* MAX_MUTUAL_SCAN_INTERVAL */
	0xDC, 0x84, /* MAX_BALANCED_SCAN_INTERVAL */
	0xDC, 0x88, /* SELF_Z_THRSH */
	0xDC, 0x8C, /* SELF_Z_MODE */
	0xDC, 0x90, /* SMART_SCAN_ENABLE */
	0xDC, 0x94, /* T_COMP_ENABLE */
	0xDC, 0x98, /* T_COMP_INTERVAL */
	0xDC, 0x9C, /* T_COMP_RECAL_MUTUAL_SENSOR_LIMIT */
	0xDC, 0xA0, /* T_COMP_RECAL_MUTUAL_HIGH */
	0xDC, 0xA4, /* T_COMP_RECAL_MUTUAL_LOW */
	0xDC, 0xA8, /* T_COMP_RECAL_SELF_SENSOR_LIMIT */
	0xDC, 0xAC, /* T_COMP_RECAL_SELF_HIGH */
	0xDC, 0xB0, /* T_COMP_RECAL_SELF_LOW */
	0xDC, 0xB4, /* CHARGER_ARMOR_ENABLE */
	0xDC, 0xB8, /* AFH_ENABLE */
	0xDC, 0xBA, /* AFH_SELF_ENABLE */
	0xDC, 0xBC, /* AFH_LISTENING_SCAN_COUNT */
	0xDC, 0xC0, /* AFH_LISTEN_SCAN_CYCLE_REPEATS */
	0xDC, 0xC4, /* AFH_LISTEN_SCAN_CONFIG */
	0xDC, 0xC8, /* CA_BLOCK_NOISE_THRESHOLD */
	0xDC, 0xCC, /* CA_BLOCK_NOISE_HYSTERESIS */
	0xDC, 0xD0, /* CA_DEFAULT_REVERT_TIME */
	0xDC, 0xD4, /* CA_SMART_H2O_REJECT */
	0xDC, 0xD6, /* CA_HOST_CONTROLLED_CHARGER */
	0xDC, 0xD8, /* T_COMP_BUTTON_MUTUAL_HIGH */
	0xDC, 0xDA, /* T_COMP_BUTTON_MUTUAL_LOW */
	0xDC, 0xDC, /* T_COMP_BUTTON_SELF_HIGH */
	0xDC, 0xDE, /* T_COMP_BUTTON_SELF_LOW */
	0xDC, 0xE0, /* CA_NUM_SUB_CONV_BASE_SELF */
	0xDC, 0xE4, /* CA_ALT_NUM_SUB_CONV_SELF */
	0xDC, 0xE8, /* BTN_SCAN_CFG */
	0xDC, 0xE9, /* Reserved233 */
	0xDC, 0xF0, /* CA_ALT_NUM_SUB_CONV_MUTUAL */
	0xDC, 0xF1, /* CA_ALT_ACQUISITION_FLAGS */
	0xDC, 0xF2, /* AFH_ALT1_TX_PERIOD */
	0xDC, 0xF3, /* AFH_ALT1_SCALE_MUT */
	0xDC, 0xF4, /* AFH_ALT1_SCALE_SELF */
	0xDC, 0xF5, /* AFH_ALT1_TX_VOLTAGE */
	0xDC, 0xF6, /* AFH_ALT1_TX_PULSES */
	0xDC, 0xF7, /* AFH_ALT2_TX_PERIOD */
	0xDC, 0xF8, /* AFH_ALT2_SCALE_MUT */
	0xDC, 0xF9, /* AFH_ALT2_SCALE_SELF */
	0xDC, 0xFA, /* AFH_ALT2_TX_VOLTAGE */
	0xDC, 0xFB, /* AFH_ALT2_TX_PULSES */
	0xDC, 0xFC, /* GEST_CFG_SIZE */
	0xDC, 0xFD, /* PAN_ACT_DSTX */
	0xDC, 0xFE, /* PAN_ACT_DSTY */
	0xDC, 0xFF, /* ZOOM_ACT_DSTX */
	0xDD, 0x00, /* ZOOM_ACT_DSTY */
	0xDD, 0x01, /* FLICK_ACT_DISTX */
	0xDD, 0x02, /* FLICK_ACT_DISTY */
	0xDD, 0x03, /* FLICK_TIME */
	0xDD, 0x04, /* ST_DEBOUNCE */
	0xDD, 0x05, /* MT_DEBOUNCE_PAN */
	0xDD, 0x06, /* MT_DEBOUNCE_ZOOM */
	0xDD, 0x07, /* MT_DEBOUNCE_P2Z */
	0xDD, 0x08, /* ROT_DEBOUNCE */
	0xDD, 0x09, /* COMPL_DEBOUNCE */
	0xDD, 0x0A, /* MT_TIMEOUT */
	0xDD, 0x0C, /* ST_DBLCLK_RMAX */
	0xDD, 0x0D, /* ST_CLICK_DISTX */
	0xDD, 0x0E, /* ST_CLICK_DISTY */
	0xDD, 0x0F, /* Reserved271 */
	0xDD, 0x10, /* MT_CLICK_TMAX */
	0xDD, 0x12, /* MT_CLICK_TMIN */
	0xDD, 0x14, /* ST_CLICK_TMAX */
	0xDD, 0x16, /* ST_CLICK_TMIN */
	0xDD, 0x18, /* ST_DBLCLK_TMAX */
	0xDD, 0x1A, /* ST_DBLCLK_TMIN */
	0xDD, 0x1C, /* GESTURE_GROUP_MASK */
	0xDD, 0x1D, /* GESTURE_GROUP1_START */
	0xDD, 0x1E, /* GESTURE_GROUP1_END */
	0xDD, 0x1F, /* GESTURE_GROUP2_START */
	0xDD, 0x20, /* GESTURE_GROUP2_END */
	0xDD, 0x21, /* GESTURE_GROUP3_START */
	0xDD, 0x22, /* GESTURE_GROUP3_END */
	0xDD, 0x23, /* GESTURE_GROUP4_START */
	0xDD, 0x24, /* GESTURE_GROUP4_END */
	0xDD, 0x25, /* Reserved293 */
	0xDD, 0x28, /* XY_FILT_CFG_SIZE */
	0xDD, 0x2C, /* XY_FILTER_MASK */
	0xDD, 0x30, /* XY_FILT_IIR_COEFF */
	0xDD, 0x34, /* XY_FILT_Z_IIR_COEFF */
	0xDD, 0x38, /* XY_FILT_XY_FAST_THR */
	0xDD, 0x39, /* XY_FILT_XY_SLOW_THR */
	0xDD, 0x3A, /* XY_FILT_IIR_FAST_COEFF */
	0xDD, 0x3B, /* Reserved315 */
	0xDD, 0x3C, /* XY_FILTER_MASK_CA */
	0xDD, 0x40, /* XY_FILT_IIR_COEFF_CA */
	0xDD, 0x44, /* XY_FILT_Z_IIR_COEFF_CA */
	0xDD, 0x48, /* XY_FILT_XY_FAST_THR_CA */
	0xDD, 0x49, /* XY_FILT_XY_SLOW_THR_CA */
	0xDD, 0x4A, /* XY_FILT_IIR_FAST_COEFF_CA */
	0xDD, 0x4B, /* Reserved331 */
	0xDD, 0x4C, /* XY_FILT_ADAPTIVE_IIR_FILTER */
	0xDD, 0x4D, /* XY_FILT_ADAPTIVE_IIR_FILTER_DISTANCE */
	0xDD, 0x4E, /* XY_FILT_TOUCH_SIZE_IIR_COEFF */
	0xDD, 0x4F, /* XY_FILT_TOUCH_SIZE_HYST */
	0xDD, 0x50, /* XY_FILT_TOUCH_ORIENTATION_IIR_COEFF */
	0xDD, 0x51, /* XY_FILT_TOUCH_ORIENTATION_HYST */
	0xDD, 0x52, /* XY_FILT_TOUCH_SCALLOPING_ENABLE */
	0xDD, 0x53, /* Reserved339 */
	0xDD, 0x5C, /* FINGER_ID_CFG_SIZE */
	0xDD, 0x60, /* Reserved352 */
	0xDD, 0x64, /* MAX_FINGER_VELOCITY */
	0xDD, 0x66, /* MAX_FINGER_VELOCITY_CA */
	0xDD, 0x68, /* LIFTOFF_DEBOUNCE */
	0xDD, 0x69, /* LIFTOFF_DEBOUNCE_CA */
	0xDD, 0x6A, /* Reserved362 */
	0xDD, 0x70, /* CENTROID_SH_CFG_SIZE */
	0xDD, 0x74, /* STYLUS_THRSH */
	0xDD, 0x78, /* STYLUS_HYST */
	0xDD, 0x7C, /* S2F_THRESHOLD */
	0xDD, 0x80, /* HOVER_THRSH */
	0xDD, 0x84, /* HOVER_HYST */
	0xDD, 0x88, /* Reserved392 */
	0xDD, 0x8C, /* ID_COORDS_CFG_SIZE */
	0xDD, 0x90, /* LRG_OBJ_CFG */
	0xDD, 0x91, /* FINGER_THRESH_MUTUAL */
	0xDD, 0x92, /* FINGER_THR_MUT_HYST */
	0xDD, 0x93, /* FINGER_THRESH_SELF */
	0xDD, 0x94, /* INNER_EDGE_GAIN */
	0xDD, 0x95, /* OUTER_EDGE_GAIN */
	0xDD, 0x96, /* X_RESOLUTION */
	0xDD, 0x98, /* Y_RESOLUTION */
	0xDD, 0x9A, /* SENSOR_ASSIGNMENT */
	0xDD, 0x9B, /* Z_SCALING */
	0xDD, 0x9C, /* RX_LINE_FILTER */
	0xDD, 0x9D, /* RX_LINE_FILTER_THRESHOLD */
	0xDD, 0x9E, /* RX_LINE_FILTER_DEBOUNCE */
	0xDD, 0x9F, /* BYPASS_THRESHOLD_GAIN */
	0xDD, 0xA0, /* BYPASS_THRESHOLD_EDGE_GAIN */
	0xDD, 0xA1, /* MAX_FAT_FINGER_SIZE */
	0xDD, 0xA2, /* MIN_FAT_FINGER_SIZE */
	0xDD, 0xA3, /* MAX_FAT_FINGER_SIZE_HYST */
	0xDD, 0xA4, /* MIN_FAT_FINGER_SIZE_HYST */
	0xDD, 0xA5, /* FAT_FINGER_COEFF */
	0xDD, 0xA6, /* FAT_FINGER_SD_THRESHOLD */
	0xDD, 0xA7, /* FAT_FINGER_COEFF_SD */
	0xDD, 0xA8, /* MULTI_TOUCH_DEBOUNCE */
	0xDD, 0xA9, /* MULTI_TOUCH_DEBOUNCE_CA */
	0xDD, 0xAA, /* FINGER_THRESHOLD_MUTUAL_CA */
	0xDD, 0xAB, /* FINGER_THRESH_SELF_CA */
	0xDD, 0xAC, /* SIZE_ORIENTATION_ENABLE */
	0xDD, 0xAD, /* MAJOR_AXIS_OFFSET */
	0xDD, 0xAE, /* MAJOR_AXIS_SCALE */
	0xDD, 0xAF, /* MINOR_AXIS_OFFSET */
	0xDD, 0xB0, /* MINOR_AXIS_SCALE */
	0xDD, 0xB1, /* CLIPPING_X_LOW */
	0xDD, 0xB2, /* CLIPPING_X_HIGH */
	0xDD, 0xB3, /* CLIPPING_Y_LOW */
	0xDD, 0xB4, /* CLIPPING_Y_HIGH */
	0xDD, 0xB5, /* CLIPPING_BOUNDARY_REMOVE */
	0xDD, 0xB6, /* Reserved438 */
	0xDD, 0xB7, /* COEF_EDGE_ATTRACTION */
	0xDD, 0xB8, /* WIDTH_CORNER_DIAG1 */
	0xDD, 0xBA, /* WIDTH_CORNER_DIAG2 */
	0xDD, 0xBC, /* WIDTH_CORNER_DIAG3 */
	0xDD, 0xBE, /* WIDTH_CORNER_DIAG4 */
	0xDD, 0xC0, /* WIDTH_CORNER_DIAG5 */
	0xDD, 0xC2, /* WIDTH_CORNER_DIAG6 */
	0xDD, 0xC4, /* WIDTH_CORNER_DIAG7 */
	0xDD, 0xC6, /* WIDTH_CORNER_DIAG8 */
	0xDD, 0xC8, /* WIDTH_CORNER_DIAG9 */
	0xDD, 0xCA, /* WIDTH_CORNER_DIAG10 */
	0xDD, 0xCC, /* WIDTH_CORNER_DIAG11 */
	0xDD, 0xCE, /* WIDTH_CORNER_PERP1 */
	0xDD, 0xD0, /* WIDTH_CORNER_PERP2 */
	0xDD, 0xD2, /* WIDTH_CORNER_PERP3 */
	0xDD, 0xD4, /* WIDTH_CORNER_PERP4 */
	0xDD, 0xD6, /* NOISE_REJECTION_3x3_FILTER_SCALE */
	0xDD, 0xD7, /* NOISE_REJECTION_3x3_FILTER_SCALE_CA */
	0xDD, 0xD8, /* DIRECT_XY_ENABLE */
	0xDD, 0xD9, /* CALC_THRESHOLD */
	0xDD, 0xDA, /* JIG_MODE_CONFIG */
	0xDD, 0xDC, /* Reserved476 */
	0xDD, 0xE4, /* BTN_CFG_SIZE */
	0xDD, 0xE8, /* BTN_THRSH_MUT_0 */
	0xDD, 0xEA, /* BTN_THRSH_MUT_1 */
	0xDD, 0xEC, /* BTN_THRSH_MUT_2 */
	0xDD, 0xEE, /* BTN_THRSH_MUT_3 */
	0xDD, 0xF0, /* BTN_HYST_MUT */
	0xDD, 0xF1, /* Reserved497 */
	0xDD, 0xF2, /* Reserved498 */
	0xDD, 0xF3, /* Reserved499 */
	0xDD, 0xF4, /* BTN_THRSH_SELF */
	0xDD, 0xF6, /* BTN_THRSH_SELF_1 */
	0xDD, 0xF8, /* BTN_THRSH_SELF_2 */
	0xDD, 0xFA, /* BTN_THRSH_SELF_3 */
	0xDD, 0xFC, /* BTN_HYST_SELF */
	0xDD, 0xFD, /* Reserved509 */
	0xDD, 0xFE, /* Reserved510 */
	0xDD, 0xFF, /* Reserved511 */
	0xDE, 0x00, /* RAW_PROC_CFG_SIZE */
	0xDE, 0x04, /* RAW_FILTER_MASK */
	0xDE, 0x06, /* RAW_FILT_IIR_COEFF_MUTUAL */
	0xDE, 0x07, /* RAW_FILT_IIR_THRESHOLD_MUTUAL */
	0xDE, 0x08, /* RAW_FILT_IIR_COEFF_SELF */
	0xDE, 0x09, /* RAW_FILT_IIR_THRESHOLD_SELF */
	0xDE, 0x0A, /* RAW_FILT_IIR_COEFF_BALANCED */
	0xDE, 0x0B, /* RAW_FILT_IIR_THRESHOLD_BALANCED */
	0xDE, 0x0C, /* RAW_FILT_IIR_COEFF_BUTTONS */
	0xDE, 0x0D, /* RAW_FILT_IIR_THRESHOLD_BUTTONS */
	0xDE, 0x0E, /* CMF_THR_MUT */
	0xDE, 0x0F, /* CMF_THR_SELF */
	0xDE, 0x10, /* CMF_THR_BTN_MUT */
	0xDE, 0x11, /* CMF_THR_BTN_SELF */
	0xDE, 0x12, /* RAW_FILTER_MASK_CA */
	0xDE, 0x14, /* RAW_FILT_IIR_COEFF_MUTUAL_CA */
	0xDE, 0x15, /* RAW_FILT_IIR_THRESHOLD_MUTUAL_CA */
	0xDE, 0x16, /* RAW_FILT_IIR_COEFF_SELF_CA */
	0xDE, 0x17, /* RAW_FILT_IIR_THRESHOLD_SELF_CA */
	0xDE, 0x18, /* RAW_FILT_IIR_COEFF_BALANCED_CA */
	0xDE, 0x19, /* RAW_FILT_IIR_THRESHOLD_BALANCED_CA */
	0xDE, 0x1A, /* RAW_FILT_IIR_COEFF_BUTTONS_CA */
	0xDE, 0x1B, /* RAW_FILT_IIR_THRESHOLD_BUTTONS_CA */
	0xDE, 0x1C, /* CMF_THR_MUT_CA */
	0xDE, 0x1D, /* CMF_THR_SELF_CA */
	0xDE, 0x1E, /* CMF_THR_BTN_MUT_CA */
	0xDE, 0x1F, /* CMF_THR_BTN_SELF_CA */
	0xDE, 0x20, /* Reserved544 */
	0xDE, 0x28, /* Reserved552 */
	0xDE, 0x2C, /* Reserved556 */
	0xDE, 0x2E, /* BL_DELAY_MUT */
	0xDE, 0x2F, /* BL_DELAY_SELF */
	0xDE, 0x30, /* BL_DELAY_BAL */
	0xDE, 0x31, /* BL_DELAY_BTN */
	0xDE, 0x32, /* BL_DELAY_MUT_CA */
	0xDE, 0x33, /* BL_DELAY_SELF_CA */
	0xDE, 0x34, /* BL_DELAY_BTN_CA */
	0xDE, 0x35, /* BL_THR_MUT */
	0xDE, 0x36, /* BL_THR_SELF */
	0xDE, 0x37, /* BL_THR_BAL */
	0xDE, 0x38, /* BL_THR_BTN_MUT */
	0xDE, 0x39, /* BL_THR_BTN_SELF */
	0xDE, 0x3A, /* BL_THR_MUT_CA */
	0xDE, 0x3B, /* BL_THR_SELF_CA */
	0xDE, 0x3C, /* BL_FILT_MUT */
	0xDE, 0x3D, /* BL_FILT_SELF */
	0xDE, 0x3E, /* BL_FILT_BAL */
	0xDE, 0x3F, /* BL_FILT_BTN_MUT */
	0xDE, 0x40, /* BL_FILT_BTN_SELF */
	0xDE, 0x41, /* CMF_THR_BAL */
	0xDE, 0x42, /* CMF_DELTA_RESET_COUNTER */
	0xDE, 0x43, /* SELF_SD_RESET */
	0xDE, 0x44, /* Reserved580 */
	0xDE, 0x50, /* H2OREJECTION_SIZE */
	0xDE, 0x54, /* BL_H20_RJCT */
	0xDE, 0x55, /* BL_H20_SNS_WIDTH */
	0xDE, 0x56, /* Reserved598 */
	0xDE, 0x58, /* CDC_CFG_SIZE */
	0xDE, 0x5C, /* TSS_CONTROL_MUT */
	0xDE, 0x60, /* TSS_LENGTH_MUT */
	0xDE, 0x64, /* TSS_TX_CONFIG_MUT */
	0xDE, 0x68, /* TSS_TX_CONTROL_MUT */
	0xDE, 0x6C, /* TSS_SEQ_CONFIG1_MUT */
	0xDE, 0x70, /* TSS_SEQ_CONFIG2_MUT */
	0xDE, 0x74, /* TSS_SEQ_CONFIG3_MUT */
	0xDE, 0x78, /* TSS_SEQ_CONFIG4_MUT */
	0xDE, 0x7C, /* TSS_SEQ_CONFIG5_MUT */
	0xDE, 0x80, /* TSS_SEQ_CONFIG6_MUT */
	0xDE, 0x84, /* TSS_SEQ_CONFIG7_MUT */
	0xDE, 0x88, /* TSS_SEQ_CONFIG8_MUT */
	0xDE, 0x8C, /* TSS_EXT_CONFIG1_MUT */
	0xDE, 0x90, /* TSS_EXT_CONFIG2_MUT */
	0xDE, 0x94, /* TSS_EXT_INTERVAL_MUT */
	0xDE, 0x98, /* TSS_INT_CONFIG1_MUT */
	0xDE, 0x9C, /* TSS_INT_CONFIG2_MUT */
	0xDE, 0xA0, /* TSS_INT_INTERVAL_MUT */
	0xDE, 0xA4, /* TSS_MCS_CONFIG_MUT */
	0xDE, 0xA8, /* TSS_RX_CONFIG_MUT */
	0xDE, 0xAC, /* Reserved684 */
	0xDE, 0xB0, /* TSS_CONTROL_SELF */
	0xDE, 0xB4, /* TSS_LENGTH_SELF */
	0xDE, 0xB8, /* TSS_TX_CONFIG_SELF */
	0xDE, 0xBC, /* TSS_TX_CONTROL_SELF */
	0xDE, 0xC0, /* TSS_SEQ_CONFIG1_SELF */
	0xDE, 0xC4, /* TSS_SEQ_CONFIG2_SELF */
	0xDE, 0xC8, /* TSS_SEQ_CONFIG3_SELF */
	0xDE, 0xCC, /* TSS_SEQ_CONFIG4_SELF */
	0xDE, 0xD0, /* TSS_SEQ_CONFIG5_SELF */
	0xDE, 0xD4, /* TSS_SEQ_CONFIG6_SELF */
	0xDE, 0xD8, /* TSS_SEQ_CONFIG7_SELF */
	0xDE, 0xDC, /* TSS_SEQ_CONFIG8_SELF */
	0xDE, 0xE0, /* TSS_EXT_CONFIG1_SELF */
	0xDE, 0xE4, /* TSS_EXT_CONFIG2_SELF */
	0xDE, 0xE8, /* TSS_EXT_INTERVAL_SELF */
	0xDE, 0xEC, /* TSS_INT_CONFIG1_SELF */
	0xDE, 0xF0, /* TSS_INT_CONFIG2_SELF */
	0xDE, 0xF4, /* TSS_INT_INTERVAL_SELF */
	0xDE, 0xF8, /* TSS_MCS_CONFIG_SELF */
	0xDE, 0xFC, /* TSS_RX_CONFIG_SELF */
	0xDF, 0x00, /* Reserved768 */
	0xDF, 0x04, /* TSS_CONTROL_BAL */
	0xDF, 0x08, /* TSS_LENGTH_BAL */
	0xDF, 0x0C, /* TSS_TX_CONFIG_BAL */
	0xDF, 0x10, /* TSS_TX_CONTROL_BAL */
	0xDF, 0x14, /* TSS_SEQ_CONFIG1_BAL */
	0xDF, 0x18, /* TSS_SEQ_CONFIG2_BAL */
	0xDF, 0x1C, /* TSS_SEQ_CONFIG3_BAL */
	0xDF, 0x20, /* TSS_SEQ_CONFIG4_BAL */
	0xDF, 0x24, /* TSS_SEQ_CONFIG5_BAL */
	0xDF, 0x28, /* TSS_SEQ_CONFIG6_BAL */
	0xDF, 0x2C, /* TSS_SEQ_CONFIG7_BAL */
	0xDF, 0x30, /* TSS_SEQ_CONFIG8_BAL */
	0xDF, 0x34, /* TSS_EXT_CONFIG1_BAL */
	0xDF, 0x38, /* TSS_EXT_CONFIG2_BAL */
	0xDF, 0x3C, /* TSS_EXT_INTERVAL_BAL */
	0xDF, 0x40, /* TSS_INT_CONFIG1_BAL */
	0xDF, 0x44, /* TSS_INT_CONFIG2_BAL */
	0xDF, 0x48, /* TSS_INT_INTERVAL_BAL */
	0xDF, 0x4C, /* TSS_MCS_CONFIG_BAL */
	0xDF, 0x50, /* TSS_RX_CONFIG_BAL */
	0xDF, 0x54, /* TSS_CONTROL_BTN */
	0xDF, 0x58, /* TSS_LENGTH_BTN_MUT */
	0xDF, 0x5C, /* TSS_TX_CONFIG_BTN */
	0xDF, 0x60, /* TSS_TX_CONTROL_BTN */
	0xDF, 0x64, /* TSS_SEQ_CONFIG1_BTN */
	0xDF, 0x68, /* TSS_SEQ_CONFIG2_BTN */
	0xDF, 0x6C, /* TSS_SEQ_CONFIG3_BTN */
	0xDF, 0x70, /* TSS_SEQ_CONFIG4_BTN */
	0xDF, 0x74, /* TSS_SEQ_CONFIG5_BTN */
	0xDF, 0x78, /* TSS_SEQ_CONFIG6_BTN */
	0xDF, 0x7C, /* TSS_SEQ_CONFIG7_BTN */
	0xDF, 0x80, /* TSS_SEQ_CONFIG8_BTN */
	0xDF, 0x84, /* TSS_EXT_CONFIG1_BTN */
	0xDF, 0x88, /* TSS_EXT_CONFIG2_BTN */
	0xDF, 0x8C, /* TSS_EXT_INTERVAL_BTN */
	0xDF, 0x90, /* TSS_INT_CONFIG1_BTN */
	0xDF, 0x94, /* TSS_INT_CONFIG2_BTN */
	0xDF, 0x98, /* TSS_INT_INTERVAL_BTN */
	0xDF, 0x9C, /* TSS_MCS_CONFIG_BTN */
	0xDF, 0xA0, /* TSS_RX_CONFIG_BTN */
	0xDF, 0xA4, /* TSS_LENGTH_BTN_SELF */
	0xDF, 0xA8, /* TSS_RX_VREF */
	0xDF, 0xAC, /* TSS_RX_LX_CONFIG */
	0xDF, 0xB0, /* TX_NUM */
	0xDF, 0xB4, /* RX_NUM */
	0xDF, 0xB8, /* SENS_NUM */
	0xDF, 0xBC, /* CROSS_NUM */
	0xDF, 0xC0, /* BUTTON_NUM */
	0xDF, 0xC4, /* SLOTS_MUT */
	0xDF, 0xC8, /* SLOTS_SELF_RX */
	0xDF, 0xCC, /* SLOTS_SELF_TX */
	0xDF, 0xD0, /* SLOTS_SELF */
	0xDF, 0xD4, /* SLOTS_BAL */
	0xDF, 0xD8, /* SCALE_MUT */
	0xDF, 0xDC, /* SCALE_SELF */
	0xDF, 0xE0, /* SCALE_BAL */
	0xDF, 0xE4, /* SCALE_BUTTON */
	0xDF, 0xE8, /* LX_MODE */
	0xDF, 0xEC, /* LX_SCALE */
	0xDF, 0xF0, /* ABSOLUTE_CR_CORRECTION_ENABLE */
	0xDF, 0xF4, /* SCANNING_MODE_MUTUAL */
	0xDF, 0xF8, /* SCANNING_MODE_BUTTON */
	0xDF, 0xFC, /* DETECT_CHARGER_THRESHOLD */
	0xDF, 0xFE, /* CA_LX_SCAN_MODE */
	0xDF, 0xFF, /* SUB_SLOT_SCAN */
	0xE0, 0x00, /* NOISE_METRIC1_THRESHOLD */
	0xE0, 0x01, /* NOISE_METRIC2_THRESHOLD */
	0xE0, 0x02, /* NOISE_METRIC3_THRESHOLD */
	0xE0, 0x03, /* AFH_DYNAMIC_THRSH_ENABLE */
	0xE0, 0x04, /* ADC_CONFIG */
	0xE0, 0x05, /* Reserved1029 */
	0xE0, 0x06, /* TSS_LDO_PROG */
	0xE0, 0x07, /* TX_PERIOD_DUMMY_SCAN */
	0xE0, 0x08, /* SINGLE_ENDED_LISTEN_SCAN */
	0xE0, 0x09, /* SPREAD_MTX */
	0xE0, 0x0A, /* SUBCONV_FILTER */
	0xE0, 0x0B, /* Reserved1035 */
	0xE0, 0x54, /* CALIBRATION_PARAM_SIZE */
	0xE0, 0x58, /* Reserved1112 */
	0xE0, 0x5C, /* Reserved1116 */
	0xE0, 0x60, /* Reserved1120 */
	0xE0, 0x64, /* Reserved1124 */
	0xE0, 0x68, /* GLOBAL_IDAC_LSB_MUTUAL */
	0xE0, 0x6C, /* GLOBAL_IDAC_LSB_SELF */
	0xE0, 0x70, /* GLOBAL_IDAC_LSB_BALANCED */
	0xE0, 0x74, /* GLOBAL_IDAC_LSB_BUTTON */
	0xE0, 0x78, /* TARGET_LEVEL_MUTUAL */
	0xE0, 0x7C, /* TARGET_LEVEL_SELF */
	0xE0, 0x80, /* TARGET_LEVEL_BALANCED */
	0xE0, 0x84, /* TARGET_LEVEL_BUTTON */
	0xE0, 0x88, /* GAIN_MUTUAL */
	0xE0, 0x8C, /* GAIN_SELF */
	0xE0, 0x90, /* GAIN_BALANCED */
	0xE0, 0x94, /* GAIN_BTN_MUTUAL */
	0xE0, 0x98, /* GAIN_BTN_SELF */
	0xE0, 0x9C, /* Reserved1180 */
	0xE0, 0xA0, /* SPREADER_CFG_SIZE */
	0xE0, 0xA4, /* CLK_IMO_SPREAD */
	0xE0, 0xA8, /* Reserved1192 */
	0xE0, 0xC0, /* CDC_PIN_INDEX_TABLE */
	0xE1, 0x01, /* Reserved1281 */
	0xE1, 0x04, /* CDC_BALANCED_LX_TABLE */
	0xE1, 0x12, /* Reserved1298 */
	0xE1, 0x18, /* CDC_BALANCED_TX_PATTERNS */
	0xE1, 0x2A, /* Reserved1322 */
	0xE1, 0x2C, /* CDC_SLOT_TABLE */
	0xE1, 0xDC, /* Reserved1500 */
	0xE1, 0xFC, /* CONFIG_CRC */
};

