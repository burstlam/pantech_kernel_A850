
typedef enum {	
	TOUCH_IOCTL_READ_LASTKEY=1001,	
	TOUCH_IOCTL_DO_KEY,	
	TOUCH_IOCTL_RELEASE_KEY, 
	TOUCH_IOCTL_CLEAN,
	TOUCH_IOCTL_DEBUG,	
	TOUCH_IOCTL_RESTART,
	TOUCH_IOCTL_PRESS_TOUCH,
	TOUCH_IOCTL_RELEASE_TOUCH,
	TOUCH_IOCTL_CHARGER_MODE,
	POWER_OFF,
	TOUCH_IOCTL_STYLUS_MODE,
	
	TOUCH_IOCTL_DELETE_ACTAREA = 2001,
	TOUCH_IOCTL_RECOVERY_ACTAREA,
	TOUCH_IOCTL_INIT = 3001,	
	TOUCH_IOCTL_OFF  = 3002,

	TOUCH_CHARGE_MODE_CTL = 4001,
	
} TOUCH_IOCTL_CMD;


