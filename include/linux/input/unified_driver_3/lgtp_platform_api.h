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
 *    File  : lgtp_platform_api.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_PLATFORM_API_H_)
#define _LGTP_PLATFORM_API_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


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
#if defined(TOUCH_DEVICE_LU201X) || defined(TOUCH_DEVICE_LU202X)
int Lu20xx_I2C_Read(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
int Lu20xx_I2C_Write(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
#endif

#if defined(TOUCH_LGD_PHASE2)
int MIT200_I2C_Read(struct i2c_client *client, u8 *cmd, int cmdLen, u8 *rxbuf, int len);
int MIT200_I2C_Write(struct i2c_client *client, u8 *cmd, int cmdLen, u8 *txbuf, int len);
#endif

#if defined(TOUCH_DEVICE_MXT2954)
int Mxt2954_I2C_Read(struct i2c_client *client, u16 cmd,  int cmdLen, void *rxbuf, int len);
int Mxt2954_I2C_Write(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
#endif

int Touch_I2C_Read_Byte(struct i2c_client *client, u8 addr, u8 *rxbuf);
int Touch_I2C_Write_Byte(struct i2c_client *client, u8 addr, u8 txbuf);
int Touch_I2C_Read(struct i2c_client *client, u8 addr, u8 *rxbuf, int len);
int Touch_I2C_Write(struct i2c_client *client, u8 addr, u8 *txbuf, int len);
int touch_i2c_read_for_query(int slave_addr, u8 *reg, u8 regLen, u8 *buf, u8 dataLen);
int TouchInitializePlatform(void);
void TouchPower(int isOn);
void TouchAssertReset(void);
void TouchResetCtrl(int isHigh);
int TouchRegisterIrq(TouchDriverData *pDriverData, irq_handler_t irqHandler);
void TouchEnableIrq(void);
void TouchDisableIrq(void);
int TouchReadMakerId(void);
int TouchReadInterrupt(void);
#if defined(TOUCH_DEVICE_LU201X) || defined(TOUCH_DEVICE_LU202X)
void TouchIntPinToggle(void);
#endif
int TouchGetBootMode(void);


#endif /* _LGTP_PLATFORM_API_H_ */

/* End Of File */

