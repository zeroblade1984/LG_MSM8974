/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  : lgtp_model_config.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_MODEL_CONFIG_H_)
#define _LGTP_MODEL_CONFIG_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/* Hardware(Board) Configuration */
#if defined(TOUCH_MODEL_Y30)

#define TOUCH_I2C_BUS_NUM 1

#define TOUCH_GPIO_RESET 0
#define TOUCH_GPIO_INTERRUPT 1
#define TOUCH_GPIO_MAKER_ID 76

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#elif defined(TOUCH_MODEL_C70)

#define TOUCH_I2C_BUS_NUM 5

#define TOUCH_GPIO_RESET			(12+902)
#define TOUCH_GPIO_INTERRUPT		(13+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_Y90)

#define TOUCH_I2C_BUS_NUM 0

#elif defined(TOUCH_MODEL_Y70)

#define TOUCH_I2C_BUS_NUM 0

#elif defined(TOUCH_MODEL_Y50)

#define TOUCH_I2C_BUS_NUM 0

#elif defined(TOUCH_MODEL_C90)

#define TOUCH_I2C_BUS_NUM 0

#elif defined(TOUCH_MODEL_C50)

#define TOUCH_I2C_BUS_NUM 5

#define TOUCH_GPIO_RESET (12+902)
#define TOUCH_GPIO_INTERRUPT (13+902)
#define TOUCH_GPIO_POWER (9+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_LION_3G)

#define TOUCH_I2C_BUS_NUM 0

#elif defined(TOUCH_MODEL_P1B)

#define TOUCH_I2C_BUS_NUM 5

#define TOUCH_GPIO_RESET			(12+902)
#define TOUCH_GPIO_INTERRUPT		(13+902)

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#elif defined(TOUCH_MODEL_T1)

#define TOUCH_I2C_BUS_NUM	5

#define TOUCH_LDO_AVDD		60
#define TOUCH_GPIO_RESET		8
#define TOUCH_GPIO_INTERRUPT	5

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#else
#error "Model should be defined"
#endif



/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
void TouchGetDeviceSpecificDriver(TouchDeviceSpecificFunction ***pDeviceFunc);
void TouchGetModelConfig(TouchDriverData *pDriverData);
void TouchPowerModel(int isOn);
void TouchAssertResetModel(void);
void TouchGetDeviceSpecificMatchTable(struct of_device_id ***pMatchtableList);

#if defined(TOUCH_DEVICE_LU201X)
extern TouchDeviceSpecificFunction Lu201x_Func;
#endif

#if defined(TOUCH_DEVICE_LU202X)
extern TouchDeviceSpecificFunction Lu202x_Func;
#endif

#if defined(TOUCH_DEVICE_FT6X36)
extern TouchDeviceSpecificFunction Ft6x36_Func;
#endif

#if defined(TOUCH_DEVICE_DUMMY)
extern TouchDeviceSpecificFunction Dummy_Func;
#endif

#if defined(TOUCH_DEVICE_S3320)
extern TouchDeviceSpecificFunction S3320_Func;
#endif

#if defined(TOUCH_DEVICE_MIT200)
extern TouchDeviceSpecificFunction MIT200_Func;
#endif

#if defined(TOUCH_MODEL_LION_3G)
extern TouchDeviceSpecificFunction td4191_Func;
#endif

#if defined(TOUCH_MODEL_T1)
extern TouchDeviceSpecificFunction mXT2954_Func;
#endif

#endif /* _LGTP_MODEL_CONFIG_H_ */

/* End Of File */

