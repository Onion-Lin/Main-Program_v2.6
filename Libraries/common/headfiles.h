#ifndef __headfiles_h
#define __headfiles_h

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_conf.h"


#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "main.h"


/* ============ common ============= */
#include "pid.h"
#include "Fun.h"
#include "DeviceCheck.h"
#include "PWM.h"

/* ============ device ============= */

#include "RC.h"

/* ============ application ======== */
#include "Ctrl.h"

#endif /* __headfiles_h */

