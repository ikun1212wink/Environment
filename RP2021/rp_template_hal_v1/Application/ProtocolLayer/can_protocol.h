/**
 *  @file       can_protocol.h
 *  @author     @RobotPilots
 *  @version    v1.1.3
 *  @brief      CAN Protocol.
 *  @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  [*]1.修改can_potocol.c/.h->can_protocol.c/.h
 *              v1.1.1(8-November-2021)
 *                  [+]1.新增can报文消息发送协议 
 *              v1.1.2(13-November-2021)
 *                  [*]1.去掉add_halfword与add_word函数中冗余的强制转换，另外
 *                       原先add_word中存在转换错误的问题，现已修复。
 *              v1.1.3(9-June-2022)
 *                  [+]1.新增对can发送端口的state和mode的控制。
 *                       当state为ENABLE时，允许消息发送；当state为DISABLE时，
 *                       禁止消息发送。
 *                       当mode为CAN_SINGLE_TX时，设置state为ENABLE之后会启动
 *                       单次传输(传输完成后会自动DISABLE)。可等待tx_state为
 *                       DISABLE之后再重新赋值为ENABLE以触发新的单次传输；
 *                       当mode为CAN_CONTINOUS_TX时，只要state为ENABLE就会周期
 *                       性连续发送。
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rm_protocol.h"
/* Exported macro ------------------------------------------------------------*/
// CAN1
#define CHASSIS_CAN_ID_LF	RM3508_CAN_ID_201
#define CHASSIS_CAN_ID_RF	RM3508_CAN_ID_202
#define CHASSIS_CAN_ID_LB	RM3508_CAN_ID_203
#define CHASSIS_CAN_ID_RB	RM3508_CAN_ID_204
// CAN2

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt);
void CAN_AddByte(drv_can_t *drv, uint8_t byte);
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword);
void CAN_AddWord(drv_can_t *drv, uint32_t word);
FunctionalState CAN_GetTxState(drv_can_t *drv);
void CAN_SetTxState(drv_can_t *drv, FunctionalState state);
void CAN_SetTxMode(drv_can_t *drv, CAN_TxModeTypeDef mode);
void CAN_ManualTx(drv_can_t *drv, uint8_t *data);

#endif
