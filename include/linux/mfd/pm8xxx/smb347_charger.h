#ifndef __SMB347_CHARGER_H
#define __SMB347_CHARGER_H

#include <linux/errno.h>
#include <linux/power_supply.h>

#define CHARGER_DEV_NAME "smb347_charger"

//20120810 so2firethread. changed.
#if defined(CONFIG_MACH_APQ8064_EF51S) || defined(CONFIG_MACH_APQ8064_EF51L)
#define USE_SMB347_CABLE_DETECTION	1
#else
#if (CONFIG_BOARD_VER == CONFIG_WS10)
#define USE_USBPHY_CABLE_DETECTION	1
#else
#define USE_SMB347_CABLE_DETECTION	1
#endif
#endif
enum battery_thermal_trip_type {
	BATT_THERM_FATAL_COLD = 0,
	BATT_THERM_CRITICAL_COLD,
	BATT_THERM_NORMAL,
	BATT_THERM_WARM,
	BATT_THERM_CRITICAL_HOT,
	BATT_THERM_FATAL_HOT,
	BATT_THERM_UNKNOWN,
};

struct pantech_therm_trip_info {
	int batt_temp;
	int therm_trip;
};

static const struct pantech_therm_trip_info batt_therm_table[] = {
		/* {temperature , battery thermal trip} */
		{-70, BATT_THERM_FATAL_COLD},
		{30, BATT_THERM_CRITICAL_COLD},
		{351, BATT_THERM_NORMAL},
		{481, BATT_THERM_WARM},
		{601, BATT_THERM_CRITICAL_HOT},
		{681, BATT_THERM_FATAL_HOT},
};

enum pantech_cable_type {
	PANTECH_CABLE_NONE=0,
	PANTECH_OTG,
	PANTECH_USB,
	PANTECH_AC,
	PANTECH_FACTORY,
	PANTECH_AUDIO_DOCKING_STATION,
	PANTECH_CABLE_MAX,
};

enum
{
	APSD_NOT_RUN	= 0,
	APSD_CDP,
	APSD_DCP,
	APSD_OCP,
	APSD_SDP,
	APSD_ACA,
	APSD_NOT_USED,
};

struct smb347_regs_value {
	u8 charge_current;
	u8 input_cur_limit; 
	u8 float_voltage;
	u8 cmd_reg_a;
	u8 cmd_reg_b;
};

struct pantech_cable_id_info {
	int cable;
	int min_adc;
	int max_adc;
};

static const struct pantech_cable_id_info pantech_cable_info[] = {
		{PANTECH_OTG, 0, 1000000},
		{PANTECH_FACTORY, 1100000, 1350000},
#ifdef CONFIG_SKY_SND_DOCKING_CRADLE		
//		{PANTECH_AUDIO_DOCKING_STATION, 1520000, 1620000},
#endif
};

#if defined(CONFIG_MACH_APQ8064_EF50L)
static const struct smb347_regs_value therm_normal_val[PANTECH_CABLE_MAX] = {
	{0x9A, 0x17, 0x6A, 0xC2, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x9A, 0x17, 0x6A, 0x91, 0x00},	/* PANTECH_OTG */
	{0x9A, 0x17, 0x6A, 0xC2, 0x02},	/* PANTECH_USB */
	{0x9A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AC */
	{0x1A, 0x19, 0x6A, 0xC2, 0x01},	/* PANTECH_FACTORY */
	{0x9A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_warm_val[PANTECH_CABLE_MAX] = {
	{0x9A, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x9A, 0x17, 0x6A, 0x91, 0x00},	/* PANTECH_OTG */
	{0x9A, 0x17, 0x6A, 0xC2, 0x02},	/* PANTECH_USB */
	{0x7A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AC */
	{0x7A, 0x19, 0x6A, 0xC2, 0x01},	/* PANTECH_FACTORY */
	{0x7A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_critical_val[PANTECH_CABLE_MAX] = {
	{0x9A, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x9A, 0x17, 0x6A, 0x91, 0x00},	/* PANTECH_OTG */
	{0x9A, 0x17, 0x6A, 0xC2, 0x02},	/* PANTECH_USB */
	{0x1A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AC */
	{0x1A, 0x19, 0x6A, 0xC2, 0x01},	/* PANTECH_FACTORY */
	{0x1A, 0x17, 0x6A, 0xC2, 0x01},	/* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_fatal_val[PANTECH_CABLE_MAX] = {
	{0x9A, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x9A, 0x17, 0x6A, 0x91, 0x00},	/* PANTECH_OTG */
	{0x9A, 0x17, 0x6A, 0xC0, 0x02},	/* PANTECH_USB */
	{0x9A, 0x17, 0x6A, 0xC0, 0x01},	/* PANTECH_AC */
	{0x9A, 0x19, 0x6A, 0xC0, 0x01},	/* PANTECH_FACTORY */
	{0x9A, 0x17, 0x6A, 0xC0, 0x01},	/* PANTECH_AUDIO_DOCKING_STATION */
};
#else
static const struct smb347_regs_value therm_normal_val[PANTECH_CABLE_MAX] = {
	{0x99, 0x17, 0x6A, 0xC2, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x99, 0x17, 0x6A, 0x91, 0x00}, 	/* PANTECH_OTG */
	{0x99, 0x17, 0x6A, 0xC2, 0x02}, /* PANTECH_USB */
	{0x99, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AC */
	{0x19, 0x19, 0x6A, 0xC2, 0x01}, /* PANTECH_FACTORY */
	{0x99, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_warm_val[PANTECH_CABLE_MAX] = {
	{0x99, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x99, 0x17, 0x6A, 0x91, 0x00}, 	/* PANTECH_OTG */
	{0x99, 0x17, 0x6A, 0xC2, 0x02}, /* PANTECH_USB */
	{0x79, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AC */
	{0x79, 0x19, 0x6A, 0xC2, 0x01}, /* PANTECH_FACTORY */
	{0x79, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_critical_val[PANTECH_CABLE_MAX] = {
	{0x99, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x99, 0x17, 0x6A, 0x91, 0x00}, 	/* PANTECH_OTG */
	{0x99, 0x17, 0x6A, 0xC2, 0x02}, /* PANTECH_USB */
	{0x19, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AC */
	{0x19, 0x19, 0x6A, 0xC2, 0x01}, /* PANTECH_FACTORY */
	{0x19, 0x17, 0x6A, 0xC2, 0x01}, /* PANTECH_AUDIO_DOCKING_STATION */
};
static const struct smb347_regs_value therm_fatal_val[PANTECH_CABLE_MAX] = {
	{0x99, 0x17, 0x6A, 0xC0, 0x01}, 	/* PANTECH_CABLE_NONE */
	{0x99, 0x17, 0x6A, 0x91, 0x00}, 	/* PANTECH_OTG */
	{0x99, 0x17, 0x6A, 0xC0, 0x02}, /* PANTECH_USB */
	{0x99, 0x17, 0x6A, 0xC0, 0x01}, /* PANTECH_AC */
	{0x99, 0x19, 0x6A, 0xC0, 0x01}, /* PANTECH_FACTORY */
	{0x99, 0x17, 0x6A, 0xC0, 0x01}, /* PANTECH_AUDIO_DOCKING_STATION */
};
#endif

struct temp_adc_info {
	int offset;
	int min_temp;
};

static const struct temp_adc_info dischg_offset_table[] = {
	{0, -110},
	{10, 270},
	{20, 510},
	{30, 600},
	{20, 700},
	{0, 900},
};

static const struct temp_adc_info dc_chg_offset_table[] = {
	{0, -110},
	{15, 330},
	{30, 510},
	{20, 600},
	{10, 700},
	{0, 900},
};

#if defined(CONFIG_PANTECH_SMB347_CHARGER)
int smb347_charger_register_vbus_sn(void (*callback)(int));
void smb347_charger_unregister_vbus_sn(void (*callback)(int));

int smb347_set_usb_power_supply_type(enum power_supply_type type);
void smb347_charger_vbus_draw(unsigned int mA, unsigned int chg_type);
#endif

#endif
