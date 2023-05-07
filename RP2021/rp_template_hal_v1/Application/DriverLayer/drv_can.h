/**
 *  @file       drv_can.h
 *  @author     @RobotPilots
 *  @version    v1.1.6
 *  @brief      CAN Driver Package(Based on HAL).
 *  @update
 *              v1.0(20-August-2020)
 *              v1.1(26-October-2021)
 *                  [*]1.修改CANx_rxDataHandler()形参canId->rxId
 *              v1.1.1(8-November-2021)
 *                  [+]1.新增邮箱机制
 *              v1.1.2(13-November-2021)
 *                  [*]1.修复创建tx_port的tx_period一直为默认值的bug
 *              v1.1.3(20-November-2021)
 *                  [+]1.CAN_MailboxTypeDef结构体增加auto_tx成员来决定邮箱是
 *                       否自动发送
 *                  [*]2.修改CAN_MailboxReadyForTx()函数的发送周期判断条件
 *                  [*]3.CAN_MailboxTypeDef结构体修改wait_tx_port_list的名称
 *                       为wait_tx_port_fifo
 *              v1.1.4(27-November-2021)
 *                  [+]1.CAN_TxPortTypeDef结构体增加delta_tx_time成员来计算
 *                       实际发送间隔
 *              v1.1.5(26-Feburary-2022)
 *                  [+]1.增加can驱动以及can邮箱的错误代码记录
 *                  [*]2.修改部分函数的变量命名，删除临时调试变量
 *                  [+]3.增加函数注释
 *              v1.1.6(9-June-2022)
 *                  [+]1.can发送端口增加state成员来决定是否允许消息发送，
 *                       增加mode成员来设置单次/连续发送(默认连续发送)
 */
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "rp_driver_config.h"
/* Exported macro ------------------------------------------------------------*/
// 请合理分配端口的发送间隔
#define MAX_WAIT_TX_PORT_CNT 4  // 等待发送的最大端口数目
/* Exported types ------------------------------------------------------------*/
typedef enum {
    CAN_MAILBOX_OK,
    CAN_MAILBOX_ERR,
    CAN_MAILBOX_BUSY
} CAN_MailboxErrnoTypeDef;

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_RxFrameTypeDef;

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_TxFrameTypeDef;

typedef struct CAN_RxPortType {
    uint32_t rx_id;
    uint8_t  rx_buff[8];
    struct CAN_RxPortType *next;
} CAN_RxPortTypeDef;

typedef struct CAN_TxPortType {
    uint32_t                tx_id;
    uint8_t                 tx_buff[8];
    uint16_t                tx_period;
    uint32_t                last_tx_time;
    uint32_t                delta_tx_time;
    FunctionalState         state;
    CAN_TxModeTypeDef       mode;
    void                    *mailbox;
    struct CAN_TxPortType   *next;
} CAN_TxPortTypeDef;

typedef struct {
    CAN_MailboxErrnoTypeDef errno;
    uint32_t            err_cnt;
    //CAN_RxPortTypeDef *rx_port;
    CAN_TxPortTypeDef   *tx_port;
    uint8_t             tx_port_cnt;
    CAN_TxPortTypeDef   *wait_tx_port_fifo[MAX_WAIT_TX_PORT_CNT];
    uint8_t             wait_tx_port_cnt;
    FunctionalState     auto_tx;
    // MailboxState
} CAN_MailboxTypeDef;

extern CAN_MailboxTypeDef hcan1Mailbox;
extern CAN_MailboxTypeDef hcan2Mailbox;

/* Exported functions --------------------------------------------------------*/
void CAN1_Init(void);
void CAN2_Init(void);
//uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);
//uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat);
//uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat);
HAL_StatusTypeDef CAN_StartTxByTxPort(CAN_TxPortTypeDef *tx_port);
HAL_StatusTypeDef CAN_StartTxByDriver(struct drv_can *drv, uint8_t *data);
void CAN_AddMsgByDriver(struct drv_can *drv, uint8_t *data, uint8_t data_cnt);
FunctionalState CAN_GetTxStateByDriver(struct drv_can *drv);
void CAN_SetTxStateByDriver(struct drv_can *drv, FunctionalState state);
void CAN_SetTxModeByDriver(struct drv_can *drv, CAN_TxModeTypeDef mode);
bool CAN_MailboxReadyForTx(CAN_MailboxTypeDef *mailbox);
void CAN_AutoTx(CAN_MailboxTypeDef *mailbox);

#endif
