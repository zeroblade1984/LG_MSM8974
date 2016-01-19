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
 *    File  : lgtp_common_driver.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_COMMON_DRIVER_H_)
#define _LGTP_COMMON_DRIVER_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#if defined(TOUCH_MODEL_Y50)
#include <dsi_drv.h>
#endif

#if defined(TOUCH_DEVICE_MXT2954)
#include <mach/lge_charging_scenario.h>
#define MONITOR_TEMP_PERIOD (60*HZ)
#define DEFAULT_TEMP        250   //BATTERY TEMP monitor
#endif
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
#if defined(TOUCH_DEVICE_MXT2954)
extern int bq24296_get_batt_temp_origin(void);
#endif

#endif /* _LGTP_COMMON_DRIVER_H_ */

/* End Of File */

