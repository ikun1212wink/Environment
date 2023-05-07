/**
 *  @file       driver.c
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Drivers' Manager.
 *  @update
 *              v1.0(9-September-2020)
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void)
{
    PWM_Init();
    ADC_Init();
    DAC_Init();
    USART1_Init();
    USART2_Init();
    USART4_Init();
    USART5_Init();
    CAN1_Init();
    CAN2_Init();	
}
