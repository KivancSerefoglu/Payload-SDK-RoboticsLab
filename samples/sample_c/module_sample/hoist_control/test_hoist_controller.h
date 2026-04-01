/**
 ********************************************************************
 * @file    test_hoist_contorller.h
 * @brief   This is the header file for "test_hoist_contorller.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2025 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HOIST_CONTROLLER_H
#define HOIST_CONTROLLER_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_psdk_hoist_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_HoistControllerStartService(void);
T_DjiReturnCode DjiTest_HoistControllerStopService(void);

#ifdef __cplusplus
}
#endif

#endif // HOIST_CONTROLLER_H

/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
