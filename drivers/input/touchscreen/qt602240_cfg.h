
#define __VER_2_0__
#define __MXT224E_CONFIG__

/* -------------------------------------------------------------------- */
/* qt602240 functional option */
/* -------------------------------------------------------------------- */
//#define QT_FIRMUP_ENABLE		// qt602240 chipset firmware update enable
#define QT_MULTITOUCH_ENABLE	// qt602240 multi touch enable
#define OPTION_WRITE_CONFIG		// ?

#define SKY_PROCESS_CMD_KEY
#define TOUCH_IO
#ifdef TOUCH_IO
#define CHARGER_MODE
#endif
//#define CHIP_NOINIT
#define RECHECK_AFTER_CALIBRATION
#define CHECK_FHE //frequency hopping errors

/* -------------------------------------------------------------------- */
/* GPIO, VREG & resolution */
/* -------------------------------------------------------------------- */
#ifdef QT_MULTITOUCH_ENABLE
#define MAX_NUM_FINGER	5
#else
#define MAX_NUM_FINGER		1
#endif

// Screen resolution
#define SCREEN_RESOLUTION_X	720//480
#define SCREEN_RESOLUTION_Y	1280//800

// Interrupt GPIO Pin
#define GPIO_TOUCH_CHG			6
#if CONFIG_BOARD_VER >= CONFIG_PT10 && !defined(CONFIG_MACH_APQ8064_EF50L)
#define GPIO_TOUCH_RST			43
#elif CONFIG_BOARD_VER >= CONFIG_WS10
#define GPIO_TOUCH_RST			43
#else
#define GPIO_TOUCH_RST			33
#endif

#define TOUCH_KEY_Y		1332
#define TOUCH_MENU_MIN		40	// 0 + 27
#define TOUCH_MENU_MAX		160	// 27 + 92
#define TOUCH_HOME_MIN		300	// 480/2 - 50
#define TOUCH_HOME_MAX		450	// 480/2 + 50
#define TOUCH_BACK_MIN		570	// 453 - 92
#define TOUCH_BACK_MAX		690	// 480 - 27

/* -------------------------------------------------------------------- */
/* qt602240 protocol define */
/* -------------------------------------------------------------------- */
#ifdef __MXT768E_CONFIG__
#define QT602240_I2C_BOOT_ADDR			0x26
#define QT602240_MAX_CHANNEL_NUM			768
#define QT602240_PAGE_NUM					13
#define QT602240_REFERENCE_MIN			4160
#define QT602240_REFERENCE_MAX			10400
#else
#define QT602240_I2C_BOOT_ADDR  			0x24
#define QT602240_MAX_CHANNEL_NUM			224
#define QT602240_PAGE_NUM					4
#define QT602240_REFERENCE_MIN			4640
#define QT602240_REFERENCE_MAX			11040
#endif

#define QT602240_PAGE_SIZE					128	
#define QT602240_DELTA_MODE			0x10
#define QT602240_REFERENCE_MODE		0x11

/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
/* DEVICE   : mxT768E CONFIGURATION */
/* -------------------------------------------------------------------- */

/* _SPT_USERDATA_T38 INSTANCE 0 */
#define T7_IDLEACQINT			64
#define T7_IDLEACQINT_PLUG		255
#define T7_ACTVACQINT			255
#define T7_ACTV2IDLETO			10

/* _GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
#define T8_CHRGTIME               	35
#define T8_TCHDRIFT             	5	
#define T8_DRIFTST              	0	
#define T8_TCHAUTOCAL            	0
#define T8_SYNC                  	0
#define T8_ATCHCALST             	9
#define T8_ATCHCALSTHR           	35
#define T8_ATCHFRCCALTHR         	50         
#define T8_ATCHFRCCALRATIO       	20      

/* _TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0 */
#define T9_CTRL				143
#define T9_XORIGIN			0
#define T9_YORIGIN			0
#define T9_XSIZE			19
#define T9_YSIZE			11
#define T9_AKSCFG			0
#define T9_BLEN				17
#define T9_TCHTHR			50
#define T9_TCHTHR_PLUG			50
#define T9_TCHDI			3
#define T9_TCHDI_PLUG			3
#define T9_ORIENT			5
#define T9_MRGTIMEOUT			0
#define T9_MOVHYSTI			10
#define T9_MOVHYSTN			2
#define T9_MOVFILTER			30
#define T9_MOVFILTER_PLUG		30
#define T9_NUMTOUCH		MAX_NUM_FINGER
#define T9_MRGHYST			10
#define T9_MRGTHR			10
#define T9_AMPHYST			10
#define T9_XRANGE			1279//1398//1398//873	
#define T9_YRANGE			719//479
#define T9_XLOCLIP			0
#define T9_XHICLIP			0
#define T9_YLOCLIP			3
#define T9_YHICLIP			3
#define T9_XEDGECTRL			0
#define T9_XEDGEDIST			0
#define T9_YEDGECTRL			0
#define T9_YEDGEDIST			0
#define T9_JUMPLIMIT			0
#define T9_TCHHYST			12  /* V2.0 or MXT224E added */
#define T9_TCHHYST_PLUG	12 /* V2.0 or MXT224E added */
#define T9_XPITCH			0  /* MXT224E added */
#define T9_YPITCH			0  /* MXT224E added */
#define T9_NEXTTCHDI		2

/* [TOUCH_KEYARRAY_T15 INSTANCE 0]    */
#define T15_CTRL                        0
#define T15_XORIGIN                     0
#define T15_YORIGIN                     0
#define T15_XSIZE                       0
#define T15_YSIZE                       0
#define T15_AKSCFG                      0
#define T15_BLEN                        0
#define T15_TCHTHR                      0
#define T15_TCHDI                       0
#define T15_RESERVED_0                  0
#define T15_RESERVED_1                  0

/*  [SPT_COMMSCONFIG_T18 INSTANCE 0]        */
#define T18_CTRL                        0
#define T18_COMMAND                     0

/* _SPT_GPIOPWM_T19 INSTANCE 0 */
#define T19_CTRL                        0
#define T19_REPORTMASK                  0
#define T19_DIR                         0
#define T19_INTPULLUP                   0
#define T19_OUT                         0
#define T19_WAKE                        0
#define T19_PWM                         0
#define T19_PERIOD                      0
#define T19_DUTY_0                      0
#define T19_DUTY_1                      0
#define T19_DUTY_2                      0
#define T19_DUTY_3                      0
#define T19_TRIGGER_0                   0
#define T19_TRIGGER_1                   0
#define T19_TRIGGER_2                   0
#define T19_TRIGGER_3                   0

/* _PROCI_GRIPFACESUPPRESSION_T20 INSTANCE 0 */
#define T20_CTRL                        0
#define T20_XLOGRIP                     0
#define T20_XHIGRIP                     0
#define T20_YLOGRIP                     0
#define T20_YHIGRIP                     0
#define T20_MAXTCHS                     0
#define T20_RESERVED_0                  0
#define T20_SZTHR1                      0
#define T20_SZTHR2                      0
#define T20_SHPTHR1                     0
#define T20_SHPTHR2                     0
#define T20_SUPEXTTO                    0

/* _PROCG_NOISESUPPRESSION_T22 INSTANCE 0 */
#define T22_CTRL					13
#define T22_RESERVED_0			0
#define T22_RESERVED_1			0
#define T22_GCAFUL             		25
#define T22_GCAFLL             		-25
#define T22_ACTVGCAFVALID		4
#define T22_NOISETHR           		30
#define T22_NOISETHR_PLUG		30
#define T22_RESERVED_2			0
#define T22_FREQHOPSCALE		0
#define T22_FREQ_0             		7
#define T22_FREQ_1             		18
#define T22_FREQ_2             		19
#define T22_FREQ_3             		23
#define T22_FREQ_4             		45

#define T22_FREQ_0_PLUG		7
#define T22_FREQ_1_PLUG             18
#define T22_FREQ_2_PLUG             19
#define T22_FREQ_3_PLUG             23
#define T22_FREQ_4_PLUG             45
#define T22_IDLEGCAFVALID		4

/* [TOUCH_PROXIMITY_T23 INSTANCE 0] */
#define T23_CTRL					0
#define T23_XORIGIN				0
#define T23_YORIGIN				0
#define T23_XSIZE				0
#define T23_YSIZE				0
#define T23_RESERVED              	0
#define T23_BLEN                  		0
#define T23_FXDDTHR               	0
#define T23_FXDDI                 		0
#define T23_AVERAGE               	0
#define T23_MVNULLRATE            	0
#define T23_MVDTHR                	0

/* T24_[PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTANCE 0] */
#define T24_CTRL					0
#define T24_NUMGEST			0
#define T24_GESTEN				0
#define T24_PROCESS				0
#define T24_TAPTO				0
#define T24_FLICKTO				0
#define T24_DRAGTO				0
#define T24_SPRESSTO			0
#define T24_LPRESSTO			0
#define T24_REPPRESSTO			0
#define T24_FLICKTHR			0
#define T24_DRAGTHR			0
#define T24_TAPTHR				0
#define T24_THROWTHR			0

/* [SPT_SELFTEST_T25 INSTANCE 0] */
#define T25_CTRL						0
#define T25_CMD                         		0
#define T25_SIGLIM_0_UPSIGLIM        	0
#define T25_SIGLIM_0_LOSIGLIM        	0
#define T25_SIGLIM_1_UPSIGLIM        	0
#define T25_SIGLIM_1_LOSIGLIM        	0
#define T25_SIGLIM_2_UPSIGLIM        	0
#define T25_SIGLIM_2_LOSIGLIM        	0

/* [PROCI_TWOTOUCHGESTUREPROCESSOR_T27 INSTANCE 0] */
#define T27_CTRL                      	0
#define T27_NUMGEST                   	0
#define T27_RESERVED_0			0
#define T27_GESTEN                    	0
#define T27_ROTATETHR                 	0
#define T27_ZOOMTHR                   	0

/* _SPT_CTECONFIG_T28 INSTANCE 0 */
#define T28_CTRL					1
#define T28_CMD					0
#define T28_MODE				1/*3*/
#define T28_IDLEGCAFDEPTH			8
#define T28_IDLEGCAFDEPTH_PLUG	8
#define T28_ACTVGCAFDEPTH			12
#define T28_ACTVGCAFDEPTH_PLUG	12
#define T28_VOLTAGE					60 


/* SPT_USERDATA_T38 INSTANCE 0 */
#define T38_USERDATA0           	0
#define T38_USERDATA1           	0 /* CAL_THR */
#define T38_USERDATA2           	0 /* num_of_antitouch */
#define T38_USERDATA3           	0 /* max touch for palm recovery  */
#define T38_USERDATA4           	0 /* MXT_ADR_T8_ATCHFRCCALRATIO for normal */
#define T38_USERDATA5           	0     
#define T38_USERDATA6           	0 
#define T38_USERDATA7           	0 /* max touch for check_auto_cal */

#define T40_CTRL                		0
#define T40_XLOGRIP             		0
#define T40_XHIGRIP             		0
#define T40_YLOGRIP             		0
#define T40_YHIGRIP				0

/* PROCI_TOUCHSUPPRESSION_T42 */

#define T42_CTRL					0
#define T42_APPRTHR				35   /* 0 (TCHTHR/4), 1 to 255 */
#define T42_MAXAPPRAREA		80   /* 0 (40ch), 1 to 255 */
#define T42_MAXTCHAREA			30   /* 0 (35ch), 1 to 255 */
#define T42_SUPSTRENGTH		170   /* 0 (128), 1 to 255 */
#define T42_SUPEXTTO			1  /* 0 (never expires), 1 to 255 (timeout in cycles) */
#define T42_MAXNUMTCHS          	0  /* 0 to 9 (maximum number of touches minus 1) */
#define T42_SHAPESTRENGTH		0  /* 0 (10), 1 to 31 */

/* T43 */
#define T43_CTRL					0
#define T43_HIDIDLERATE			0
#define T43_XLENGTH				0
#define T43_YLENGTH				0
#define T43_RWKRATE			0	

/* SPT_CTECONFIG_T46  */
#define T46_CTRL					0     /*Reserved */
#define T46_MODE				3     /*0: 16X14Y, 1: 17X13Y, 2: 18X12Y, 3: 19X11Y, 4: 20X10Y, 5: 21X15Y, 6: 22X8Y, */
#define T46_IDLESYNCSPERX		16
#define T46_ACTVSYNCSPERX		48
#define T46_ADCSPERSYNC		0 
#define T46_PULSESPERADC		0     /*0:1  1:2   2:3   3:4 pulses */
#define T46_XSLEW				1    /*0:500nsec,  1:350nsec */
#define T46_SYNCDELAY			0 

/* PROCI_STYLUS_T47 */              
#define T47_CTRL					0
#define T47_CONTMIN				22
#define T47_CONTMAX			34
#define T47_STABILITY			10
#define T47_MAXTCHAREA          	5
#define T47_AMPLTHR				30
#define T47_STYSHAPE			2
#define T47_HOVERSUP			230
#define T47_CONFTHR				2
#define T47_SYNCSPERX           	0

/* PROCG_NOISESUPPRESSION_T48  */
#define T48_CTRL			1  
#define T48_CFG 			4  
#define T48_CFG_PLUG		4
#define T48_CALCFG			80
#define T48_CALCFG_PLUG 		80
#define T48_BASEFREQ			0  
#define T48_RESERVED0           	0  
#define T48_RESERVED1           	0  
#define T48_RESERVED2           	0  
#define T48_RESERVED3           	0  
#define T48_MFFREQ_2			0  
#define T48_MFFREQ_3			0 
#define T48_RESERVED4           	0  
#define T48_RESERVED5           	0  
#define T48_RESERVED6           	0  
#define T48_GCACTVINVLDADCS		6
#define T48_GCIDLEINVLDADCS		6  
#define T48_RESERVED7           	0  
#define T48_RESERVED8           	0  
#define T48_GCMAXADCSPERX       	100
#define T48_GCLIMITMIN          	4
#define T48_GCLIMITMAX          	64 
#define T48_GCCOUNTMINTGT       	10 
#define T48_MFINVLDDIFFTHR      	0 
#define T48_MFINCADCSPXTHR      	5
#define T48_MFERRORTHR          	38 
#define T48_SELFREQMAX          	20
#define T48_RESERVED9           	0  
#define T48_RESERVED10          	0  
#define T48_RESERVED11          	0  
#define T48_RESERVED12          	0  
#define T48_RESERVED13          	0  
#define T48_RESERVED14          	0  
#define T48_BLEN                	0  
#define T48_TCHTHR			45
#define T48_TCHDI			2
#define T48_MOVHYSTI			3
#define T48_MOVHYSTN            	1 
#define T48_MOVFILTER           	11  
#define T48_NUMTOUCH            	5  
#define T48_MRGHYST			5 
#define T48_MRGTHR			5
#define T48_XLOCLIP			-10  
#define T48_XHICLIP			-10  
#define T48_YLOCLIP			10 
#define T48_YHICLIP			10 
#define T48_XEDGECTRL           	138
#define T48_XEDGEDIST           	60
#define T48_YEDGECTRL           	128
#define T48_YEDGEDIST           	0
#define T48_JUMPLIMIT			10
#define T48_TCHHYST			15 
#define T48_NEXTTCHDI			0

#define T52_CTRL					0
#define T52_XORIGIN				0
#define T52_YORIGIN				0
#define T52_RESERVED0			0
#define T52_RESERVED1			0
#define T52_AKSCFG				0
#define T52_RESERVED2			0
#define T52_FXDDTHR				0
#define T52_FXDDI				0
#define T52_AVERAGE				0
#define T52_MVNULLRATE			0
#define T52_MVDTHR				0

#define T53_DSTYPE				1
#define T53_XORIGIN				32
#define T53_XSIZE				1
#define T53_YORIGIN				0
#define T53_YSIZE				2

/*
#ifdef CHECK_FHE
typedef enum
{
	FHE_CLEAR_TCHTHR 		= 		CHARGER_UNPLUGGED_TCHTHR,
	FHE_CLEAR_TCHDI 		= 		CHARGER_UNPLUGGED_TCHDI,
	FHE_CLEAR_IDLEGCAFDEPTH = 		CHARGER_UNPLUGGED_IDLEGCAFDEPTH,
	FHE_CLEAR_ACTVGCAFDEPTH = 		CHARGER_UNPLUGGED_ACTVGCAFDEPTH,
	FHE_CLEAR_NOISETHR 		= 		CHARGER_UNPLUGGED_NOISETHR,
	FHE_CLEAR_IDLEACQINT	= 		CHARGER_UNPLUGGED_IDLEACQINT, 
	FHE_CLEAR_ACTVACQINT 	= 		CHARGER_UNPLUGGED_ACTVACQINT, 

	FHE_SET_TCHTHR 			= 		CHARGER_PLUGGED_TCHTHR,
	FHE_SET_TCHDI 			= 		CHARGER_PLUGGED_TCHDI,
	FHE_SET_IDLEGCAFDEPTH 	= 		CHARGER_PLUGGED_IDLEGCAFDEPTH,
	FHE_SET_ACTVGCAFDEPTH 	= 		CHARGER_PLUGGED_ACTVGCAFDEPTH,
	FHE_SET_NOISETHR 		= 		CHARGER_PLUGGED_NOISETHR,
	FHE_SET_IDLEACQINT 		= 		CHARGER_PLUGGED_IDLEACQINT, 
	FHE_SET_ACTVACQINT 		= 		CHARGER_PLUGGED_ACTVACQINT,
} FHE_ARG;
#endif
*/

typedef enum
{
	STYLUS_UNPLUGGED_T9_TCHTHR	= 90,
	STYLUS_PLUGGED_T9_TCHTHR	= 50,
	NORMAL_UNPLUGGED_T9_TCHTHR	= T9_TCHTHR,
	NORMAL_PLUGGED_T9_TCHTHR	= T9_TCHTHR_PLUG,

	STYLUS_UNPLUGGED_T9_MOVHYSTI = 0,
	STYLUS_PLUGGED_T9_MOVHYSTI	= 3,
	NORMAL_UNPLUGGED_T9_MOVHYSTI = T9_MOVHYSTI,
	NORMAL_PLUGGED_T9_MOVHYSTI	= T9_MOVHYSTI,

	STYLUS_UNPLUGGED_T9_MOVHYSTN = 0,
	STYLUS_PLUGGED_T9_MOVHYSTN	= 1,
	NORMAL_UNPLUGGED_T9_MOVHYSTN = T9_MOVHYSTN,
	NORMAL_PLUGGED_T9_MOVHYSTN	= T9_MOVHYSTN,

	STYLUS_UNPLUGGED_T9_MOVFILTER = 0,
	STYLUS_PLUGGED_T9_MOVFILTER	= 30,
	NORMAL_UNPLUGGED_T9_MOVFILTER = T9_MOVFILTER,
	NORMAL_PLUGGED_T9_MOVFILTER	= T9_MOVFILTER_PLUG,

	STYLUS_UNPLUGGED_T9_JUMPLIMIT = 8,
	STYLUS_PLUGGED_T9_JUMPLIMIT	= 10,
	NORMAL_UNPLUGGED_T9_JUMPLIMIT = T9_JUMPLIMIT,
	NORMAL_PLUGGED_T9_JUMPLIMIT	= T9_JUMPLIMIT,

	STYLUS_UNPLUGGED_T42_CTRL = 0,
	STYLUS_PLUGGED_T42_CTRL	= 0,
	NORMAL_UNPLUGGED_T42_CTRL = T42_CTRL,
	NORMAL_PLUGGED_T42_CTRL	= T42_CTRL,

	STYLUS_UNPLUGGED_T47_CTRL = 1,
	STYLUS_PLUGGED_T47_CTRL	= 1,
	NORMAL_UNPLUGGED_T47_CTRL = T47_CTRL,
	NORMAL_PLUGGED_T47_CTRL	= T47_CTRL,

	STYLUS_UNPLUGGED_T47_AMPLTHR = 50,
	STYLUS_PLUGGED_T47_AMPLTHR	= 50,
	NORMAL_UNPLUGGED_T47_AMPLTHR = T47_AMPLTHR,
	NORMAL_PLUGGED_T47_AMPLTHR	= T47_AMPLTHR,

	STYLUS_UNPLUGGED_T48_CTRL = 3,
	STYLUS_PLUGGED_T48_CTRL	= 19,
	NORMAL_UNPLUGGED_T48_CTRL = T48_CTRL,
	NORMAL_PLUGGED_T48_CTRL	= T48_CTRL,

	STYLUS_UNPLUGGED_T48_CALCFG = T48_CALCFG,
	STYLUS_PLUGGED_T48_CALCFG	= T48_CALCFG_PLUG,
	NORMAL_UNPLUGGED_T48_CALCFG = T48_CALCFG,
	NORMAL_PLUGGED_T48_CALCFG	= T48_CALCFG_PLUG,

	STYLUS_UNPLUGGED_T48_CFG = 0,
	STYLUS_PLUGGED_T48_CFG	= 0,
	NORMAL_UNPLUGGED_T48_CFG = T48_CFG,
	NORMAL_PLUGGED_T48_CFG	= T48_CFG,

	STYLUS_UNPLUGGED_T48_MFERRORTHR = 48,
	STYLUS_PLUGGED_T48_MFERRORTHR	= 46,
	NORMAL_UNPLUGGED_T48_MFERRORTHR = T48_MFERRORTHR,
	NORMAL_PLUGGED_T48_MFERRORTHR	= T48_MFERRORTHR,

	STYLUS_UNPLUGGED_T48_SELFREQMAX = 20,
	STYLUS_PLUGGED_T48_SELFREQMAX	= 5,
	NORMAL_UNPLUGGED_T48_SELFREQMAX = T48_SELFREQMAX,
	NORMAL_PLUGGED_T48_SELFREQMAX	= T48_SELFREQMAX,

	STYLUS_UNPLUGGED_T48_BLEN = 160,
	STYLUS_PLUGGED_T48_BLEN	= 144,
	NORMAL_UNPLUGGED_T48_BLEN = T48_BLEN,
	NORMAL_PLUGGED_T48_BLEN	= T48_BLEN,

	STYLUS_UNPLUGGED_T48_TCHTHR = T48_TCHTHR,
	STYLUS_PLUGGED_T48_TCHTHR	= 100,
	NORMAL_UNPLUGGED_T48_TCHTHR = T48_TCHTHR,
	NORMAL_PLUGGED_T48_TCHTHR	= T48_TCHTHR,

	STYLUS_UNPLUGGED_T48_MOVHYSTI = T48_MOVHYSTI,
	STYLUS_PLUGGED_T48_MOVHYSTI	= 0,
	NORMAL_UNPLUGGED_T48_MOVHYSTI = T48_MOVHYSTI,
	NORMAL_PLUGGED_T48_MOVHYSTI	= T48_MOVHYSTI,

	STYLUS_UNPLUGGED_T48_MOVHYSTN = T48_MOVHYSTN,
	STYLUS_PLUGGED_T48_MOVHYSTN	= 0,
	NORMAL_UNPLUGGED_T48_MOVHYSTN = T48_MOVHYSTN,
	NORMAL_PLUGGED_T48_MOVHYSTN	= T48_MOVHYSTN,	
	
	STYLUS_UNPLUGGED_T48_MRGTHR = 40,
	STYLUS_PLUGGED_T48_MRGTHR	= 5,
	NORMAL_UNPLUGGED_T48_MRGTHR = T48_MRGTHR,
	NORMAL_PLUGGED_T48_MRGTHR	= T48_MRGTHR,

	STYLUS_UNPLUGGED_T48_XLOCLIP = 0,
	STYLUS_PLUGGED_T48_XLOCLIP	= 5,
	NORMAL_UNPLUGGED_T48_XLOCLIP = T48_XLOCLIP,
	NORMAL_PLUGGED_T48_XLOCLIP	= T48_XLOCLIP,

	STYLUS_UNPLUGGED_T48_XHICLIP = 0,
	STYLUS_PLUGGED_T48_XHICLIP	= 5,
	NORMAL_UNPLUGGED_T48_XHICLIP = T48_XHICLIP,
	NORMAL_PLUGGED_T48_XHICLIP	= T48_XHICLIP,

	STYLUS_UNPLUGGED_T48_YLOCLIP = 0,
	STYLUS_PLUGGED_T48_YLOCLIP	= 10,
	NORMAL_UNPLUGGED_T48_YLOCLIP = T48_YLOCLIP,
	NORMAL_PLUGGED_T48_YLOCLIP	= T48_YLOCLIP,	

	STYLUS_UNPLUGGED_T48_YHICLIP = 0,
	STYLUS_PLUGGED_T48_YHICLIP	= 10,
	NORMAL_UNPLUGGED_T48_YHICLIP = T48_YHICLIP,
	NORMAL_PLUGGED_T48_YHICLIP	= T48_YHICLIP,

	STYLUS_UNPLUGGED_T48_YEDGEDIST = 0,
	STYLUS_PLUGGED_T48_YEDGEDIST	= 35,
	NORMAL_UNPLUGGED_T48_YEDGEDIST = T48_YEDGEDIST,
	NORMAL_PLUGGED_T48_YEDGEDIST	= T48_YEDGEDIST,

	STYLUS_UNPLUGGED_T48_JUMPLIMIT = 0,
	STYLUS_PLUGGED_T48_JUMPLIMIT	= 8,
	NORMAL_UNPLUGGED_T48_JUMPLIMIT = T48_JUMPLIMIT,
	NORMAL_PLUGGED_T48_JUMPLIMIT	= T48_JUMPLIMIT,

	STYLUS_UNPLUGGED_T48_NEXTTCHDI = 3,
	STYLUS_PLUGGED_T48_NEXTTCHDI	= 0,
	NORMAL_UNPLUGGED_T48_NEXTTCHDI = T48_NEXTTCHDI,
	NORMAL_PLUGGED_T48_NEXTTCHDI	= T48_NEXTTCHDI,	
} STYLUS_SET;


