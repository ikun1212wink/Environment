/**
 *  @file       driver.h
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Drivers' Manager.
 *  @update
 *              v1.0(9-September-2020)
 */
#ifndef __DRIVER_H
#define __DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#include "drv_haltick.h"    // v1.2    
#include "drv_can.h"        // v1.1.6
#include "drv_tim.h"        // v1.0
#include "drv_uart.h"       // v1.2
#include "drv_io.h"         // v1.0
#include "drv_adda.h"       // v1.0

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void);

#endif
