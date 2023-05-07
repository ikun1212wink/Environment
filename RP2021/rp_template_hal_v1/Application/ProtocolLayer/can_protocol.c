/**
 *  @file       can_protocol.c
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
 
/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

#include "drv_can.h"
#include "chassis_motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *  @brief  CAN 添加消息
 */
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_AddMsgByDriver(drv, data, data_cnt);
}

/**
 *  @brief  CAN 添加单字节数据(Byte)
 */
void CAN_AddByte(drv_can_t *drv, uint8_t byte)
{
    CAN_AddMsgByDriver(drv, &byte, 1);
}

/**
 *  @brief  CAN 添加半字数据(HalfWord)
 */
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword)
{
    uint8_t data[2];
    data[0] = (uint8_t)(hword >> 8);
    data[1] = (uint8_t)(hword);
    CAN_AddMsgByDriver(drv, data, 2);
}

/**
 *  @brief  CAN 添加字数据(Word)
 */
void CAN_AddWord(drv_can_t *drv, uint32_t word)
{
    uint8_t data[4];
    data[0] = (uint8_t)(word >> 24);
    data[1] = (uint8_t)(word >> 16);
    data[2] = (uint8_t)(word >> 8);
    data[3] = (uint8_t)(word);
    CAN_AddMsgByDriver(drv, data, 4);
}

/**
 *  @brief  CAN 获取发送端口的使能状态
 */
FunctionalState CAN_GetTxState(drv_can_t *drv)
{
    return CAN_GetTxStateByDriver(drv);
}

/**
 *  @brief  CAN 使能/失能发送端口
 */
void CAN_SetTxState(drv_can_t *drv, FunctionalState state)
{
    CAN_SetTxStateByDriver(drv, state);
}

/**
 *  @brief  CAN 设置发送端口为单次/连续发送
 */
void CAN_SetTxMode(drv_can_t *drv, CAN_TxModeTypeDef mode)
{
    CAN_SetTxModeByDriver(drv, mode);
}

/**
 *  @brief  CAN 立即发送报文
 */
void CAN_ManualTx(drv_can_t *drv, uint8_t *data)
{
    CAN_StartTxByDriver(drv, data);
}

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    /* 左前轮 */
    if(rxId == CHASSIS_CAN_ID_LF)
    {
        // 更新底盘电机数据
        chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], rxBuf);
        chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
    }
    /* 右前轮 */
    else if(rxId == CHASSIS_CAN_ID_RF)
    {
        // 更新底盘电机数据
        chassis_motor[CHAS_RF].update(&chassis_motor[CHAS_RF], rxBuf);
        chassis_motor[CHAS_RF].check(&chassis_motor[CHAS_RF]);
    }
    /* 左后轮 */
    else if(rxId == CHASSIS_CAN_ID_LB)
    {
        // 更新底盘电机数据
        chassis_motor[CHAS_LB].update(&chassis_motor[CHAS_LB], rxBuf);
        chassis_motor[CHAS_LB].check(&chassis_motor[CHAS_LB]);
    }
    /* 右后轮 */
    else if(rxId == CHASSIS_CAN_ID_RB)
    {
        // 更新底盘电机数据
        chassis_motor[CHAS_RB].update(&chassis_motor[CHAS_RB], rxBuf);
        chassis_motor[CHAS_RB].check(&chassis_motor[CHAS_RB]);
    }
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

