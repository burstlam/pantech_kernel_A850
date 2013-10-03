#ifndef _TCHKEYPT_H_
#define _TCHKEYPT_H_

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

struct tchkeyptdata_t{
	struct input_dev *tchkeypt;
	struct i2c_client	*client;
	struct work_struct work;
	struct mutex		i2clock;
	unsigned int new_keyvalue;
	unsigned int MSM_STATUS;
	unsigned int PAD_FUNCTION;
	unsigned int PAD_TEMP_FUNCTION;
	unsigned int PAD_keyvalue;
	atomic_t enable;
	spinlock_t lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};


#endif // _TCHKEYPT_H_
