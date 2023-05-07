/**
 * @file        drv_tim.h
 * @author      @RobotPilots
 * @version     v1.0       
 * @brief       TIMER Driver Package(Based on HAL).
 * @update      v1.0(23-August-2020)
 */
#ifndef __DRV_TIM_H
#define __DRV_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
#define FRIC_PWM_L   TIM3->CCR1
#define FRIC_PWM_R   TIM3->CCR2
#define SERVO_PWM    TIM1->CCR2

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);

#endif
