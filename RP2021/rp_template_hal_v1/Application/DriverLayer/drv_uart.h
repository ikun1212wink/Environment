/**
 *  @file       drv_uart.h
 *  @author     @RobotPilots
 *  @version    v1.2
 *  @brief      UART Driver Package(Based on HAL).
 *  @update     
 *              v1.0(15-August-2020)
 *              v1.1(1-Feburary-2022)
 *                  [+]1.增加uart驱动的函数实现
 *              v1.2(26-Feburary-2022)
 *                  [+]1.增加uart驱动错误代码的记录
 */
#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "rp_driver_config.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
// USARTx Init
void USART1_Init(void);
void USART2_Init(void);
void USART4_Init(void);
void USART5_Init(void);
// drv_can
void USART_AddMsg(drv_uart_t *drv, uint8_t *data, uint8_t data_cnt);
void USART_StartTx(drv_uart_t *drv);
void USART_StartTxDma(drv_uart_t *drv);

#endif
