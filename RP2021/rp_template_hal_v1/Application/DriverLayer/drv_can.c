/**
 *  @file       drv_can.c
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

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "cmsis_os.h"
#include "rp_math.h"
#include "drv_haltick.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Private macro -------------------------------------------------------------*/
//#define DEFAULT_CAN_TX_PERIOD (2)   // 选择自动发送时建议发送间隔>=2ms

/* Private function prototypes -----------------------------------------------*/
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig);
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;

/* Exported variables --------------------------------------------------------*/
CAN_MailboxTypeDef hcan1Mailbox = {
    .errno = CAN_MAILBOX_OK,
    .err_cnt = 0,
    .tx_port = NULL,
    .tx_port_cnt = 0,
    .wait_tx_port_fifo = {NULL,NULL,NULL,NULL},
    .wait_tx_port_cnt = 0,
    .auto_tx = ENABLE
};

CAN_MailboxTypeDef hcan2Mailbox = {
    .errno = CAN_MAILBOX_OK,
    .err_cnt = 0,
    .tx_port = NULL,
    .tx_port_cnt = 0,
    .wait_tx_port_fifo = {NULL,NULL,NULL,NULL},
    .wait_tx_port_cnt = 0,
    .auto_tx = ENABLE
};

/* Private functions ---------------------------------------------------------*/
/**
 *  @brief  CAN 标识符过滤器复位成默认配置
 */
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
    sFilterConfig->FilterIdHigh = 0;                        
    sFilterConfig->FilterIdLow = 0;                            
    sFilterConfig->FilterMaskIdHigh = 0;                    // 不过滤
    sFilterConfig->FilterMaskIdLow = 0;                     // 不过滤
    sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0; // 过滤器关联到FIFO0
    sFilterConfig->FilterBank = 0;                          // 设置过滤器0
    sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;      // 标识符屏蔽模式
    sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;     // 32位宽
    sFilterConfig->FilterActivation = ENABLE;               // 激活滤波器
    sFilterConfig->SlaveStartFilterBank = 0;
}

/**
 *  @brief  CAN 接收中断回调函数
 */
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
        
        CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
    }
    else if(hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
        
        CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
    }
}

/**
 *  @brief  CAN 遍历邮箱的发送端口，若存在则返回端口地址
 */
static CAN_TxPortTypeDef* CAN_GetTxPort(CAN_MailboxTypeDef *mailbox, uint32_t tx_id)
{
    bool isFound = false;
    CAN_TxPortTypeDef *port = mailbox->tx_port;
    
    while(port != NULL) {
        if(port->tx_id == tx_id) {
            isFound = true;
            break;
        }
        port = port->next;
    }
    
    if( !isFound )
        port = NULL;
    
    return port;
}

/**
 *  @brief  CAN 创建发送端口
 */
static CAN_TxPortTypeDef* CAN_CreateTxPort(CAN_MailboxTypeDef *mailbox, drv_can_t *drv)
{
    CAN_TxPortTypeDef *port = mailbox->tx_port;
    
    if((port != NULL) && (drv != NULL)) {
        // find the last port
        while(port->next != NULL)
            port = port->next;
        // create a new port(tx_buff fill zero)
        port->next = (CAN_TxPortTypeDef *)pvPortMalloc(sizeof(CAN_MailboxTypeDef));
        port->next->tx_id = drv->tx_id;
        port->next->tx_period = drv->tx_period;
        port->next->last_tx_time = 0;
        port->next->delta_tx_time = 0;
        port->next->state = ENABLE;
        port->next->mode = CAN_CONTINOUS_TX;
        for(uint8_t i = 0; i < 8; i++)
            port->next->tx_buff[i] = 0;
        port->next->next = NULL;
        port->next->mailbox = mailbox;
        // return the last created port
        port = port->next;
    }
    else {
        // create a new port(tx_buff fill zero)
        mailbox->tx_port = (CAN_TxPortTypeDef *)pvPortMalloc(sizeof(CAN_MailboxTypeDef));
        mailbox->tx_port->tx_id = drv->tx_id;
        mailbox->tx_port->tx_period = drv->tx_period;
        mailbox->tx_port->last_tx_time = 0;
        mailbox->tx_port->delta_tx_time = 0;
        mailbox->tx_port->state = ENABLE;
        mailbox->tx_port->mode = CAN_CONTINOUS_TX;
        for(uint8_t i = 0; i < 8; i++)
            mailbox->tx_port->tx_buff[i] = 0;
        mailbox->tx_port->next = NULL;
        mailbox->tx_port->mailbox = mailbox;
        // return the last created port
        port = mailbox->tx_port;
    }
    
    mailbox->tx_port_cnt += 1;
    
    return port;
}

/**
 *  @brief  CAN 添加消息到发送端口
 */
static HAL_StatusTypeDef CAN_AddMsgToTxPort(CAN_TxPortTypeDef *port, uint8_t *data, uint8_t data_cnt, uint8_t start_idx)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    for(uint8_t i = 0; i < data_cnt; i++) {
        if((start_idx+i) >= 8) {
            // @Error: out of range(must < 8)
            ret = HAL_ERROR;
            break;
        }
        port->tx_buff[start_idx+i] = data[i];
    }
    
    return ret;
}

/**
 *  @brief  CAN 更新发送端口的发送间隔(取最短间隔)
 */
static void CAN_UpdatePeriodAtTxPort(CAN_TxPortTypeDef *port, uint16_t tx_period)
{
    if(tx_period < port->tx_period)
        port->tx_period = tx_period;
}

/**
 *  @brief  CAN 待发送端口FIFO的出队
 */
static CAN_TxPortTypeDef* CAN_PopWaitTxPort(CAN_MailboxTypeDef *mailbox)
{
    CAN_TxPortTypeDef* tx_port = NULL;
    
    if(mailbox->wait_tx_port_cnt > 0) {
        tx_port = mailbox->wait_tx_port_fifo[0];
        for(uint8_t i = 0; i < MAX_WAIT_TX_PORT_CNT-1; i++) {
            mailbox->wait_tx_port_fifo[i] = mailbox->wait_tx_port_fifo[i+1];
        }
        mailbox->wait_tx_port_fifo[MAX_WAIT_TX_PORT_CNT-1] = NULL;
        mailbox->wait_tx_port_cnt -= 1;
    }
    
    return tx_port;
}

/**
 *  @brief  CAN 待发送端口FIFO的入队
 */
static HAL_StatusTypeDef CAN_PushWaitTxPort(CAN_MailboxTypeDef *mailbox, CAN_TxPortTypeDef *tx_port)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    
    if(mailbox->wait_tx_port_cnt < MAX_WAIT_TX_PORT_CNT) {
        mailbox->wait_tx_port_fifo[mailbox->wait_tx_port_cnt] = tx_port;
        mailbox->wait_tx_port_cnt++;
        ret = HAL_OK;
    } else {
        mailbox->errno = CAN_MAILBOX_BUSY;
        mailbox->err_cnt += 1;
    }
    
    return ret;
}

/* Exported functions --------------------------------------------------------*/
/**
 *  @brief  CAN1 初始化
 */
void CAN1_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    // 配置CAN标识符滤波器
    CAN_Filter_ParamsInit(&sFilterConfig);
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    
    // 使能接收中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // 开启CAN1
    HAL_CAN_Start(&hcan1);
}

/**
 *  @brief  CAN2 初始化
 */
void CAN2_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    // 配置CAN标识符滤波器
    CAN_Filter_ParamsInit(&sFilterConfig);
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    
    // 使能接收中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // 开启CAN2
    HAL_CAN_Start(&hcan2);
}

/**
 *  @brief  根据drv_can将消息添加到对应邮箱的发送端口，若端口不存在则自动创建。
 *          邮箱会定时查询发送端口是否就绪（发送间隔是否已到），若端口就绪，则
 *          消息将通过实际的CAN口发送。
 */
void CAN_AddMsgByDriver(struct drv_can *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    CAN_MailboxTypeDef *mailbox = NULL;    
     
    if(drv->id == DRV_CAN1) {
        mailbox = &hcan1Mailbox;
    }
    else if(drv->id == DRV_CAN2) {
        mailbox = &hcan2Mailbox;
    }
    else {
        // unknown can_id
        drv->errno = DRV_UNKNOWN_ID;
        drv->err_cnt += 1;
        return;
    }
    
    /* get tx_port by mailbox */
    tx_port = CAN_GetTxPort(mailbox, drv->tx_id);
    /* port not found */
    if(tx_port == NULL) {
        tx_port = CAN_CreateTxPort(mailbox, drv);
    }
    /* add msg to corresponding txport */
    if(HAL_ERROR == CAN_AddMsgToTxPort(tx_port, data, data_cnt, drv->data_idx)) {
        drv->errno = DRV_OUT_OF_RANGE;
        drv->err_cnt += 1;
    }
    /* set the lowest tx_period as the period of the tx_port */
    CAN_UpdatePeriodAtTxPort(tx_port, drv->tx_period);
}

/**
 *  @brief  根据drv_can返回对应邮箱发送端口的使能状态
 */
FunctionalState CAN_GetTxStateByDriver(struct drv_can *drv)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    CAN_MailboxTypeDef *mailbox = NULL;    
     
    if(drv->id == DRV_CAN1) {
        mailbox = &hcan1Mailbox;
    }
    else if(drv->id == DRV_CAN2) {
        mailbox = &hcan2Mailbox;
    }
    else {
        // unknown can_id
        drv->errno = DRV_UNKNOWN_ID;
        drv->err_cnt += 1;
        return DISABLE;
    }
    
    /* get tx_port by mailbox */
    tx_port = CAN_GetTxPort(mailbox, drv->tx_id);
    /* port not found */
    if(tx_port == NULL) {
        tx_port = CAN_CreateTxPort(mailbox, drv);
    }
    /* return tx_port->state */
    return tx_port->state;
}

/**
 *  @brief  根据drv_can设置对应邮箱发送端口的使能状态，若端口不存在则自动创建。
 *          如果端口使能(ENABLE)，则端口就绪后消息会进入邮箱的待发送列表。
 *          如果端口失能(DISABLE)，即便端口就绪，消息都不会进入邮箱的待发送列表。
 *          简言之就是端口消息不会被发送。
 */
void CAN_SetTxStateByDriver(struct drv_can *drv, FunctionalState state)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    CAN_MailboxTypeDef *mailbox = NULL;    
     
    if(drv->id == DRV_CAN1) {
        mailbox = &hcan1Mailbox;
    }
    else if(drv->id == DRV_CAN2) {
        mailbox = &hcan2Mailbox;
    }
    else {
        // unknown can_id
        drv->errno = DRV_UNKNOWN_ID;
        drv->err_cnt += 1;
        return;
    }
    
    /* get tx_port by mailbox */
    tx_port = CAN_GetTxPort(mailbox, drv->tx_id);
    /* port not found */
    if(tx_port == NULL) {
        tx_port = CAN_CreateTxPort(mailbox, drv);
    }
    /* set tx_port->state to new state(ENABLE/DISABLE) */
    tx_port->state = state;
}

/**
 *  @brief  根据drv_can设置对应邮箱发送端口的发送模式，包括单次发送和连续发送。
 *          在单次发送模式下，需要先使能发送端口以启动消息发送。当端口消息发送
 *          完成之后会自动失能端口，从而达到单次发送的目的。
 *          在连续发送模式下，只要端口已使能，就会周期性连续发送。
 */
void CAN_SetTxModeByDriver(struct drv_can *drv, CAN_TxModeTypeDef mode)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    CAN_MailboxTypeDef *mailbox = NULL;    
     
    if(drv->id == DRV_CAN1) {
        mailbox = &hcan1Mailbox;
    }
    else if(drv->id == DRV_CAN2) {
        mailbox = &hcan2Mailbox;
    }
    else {
        // unknown can_id
        drv->errno = DRV_UNKNOWN_ID;
        drv->err_cnt += 1;
        return;
    }
    
    /* get tx_port by mailbox */
    tx_port = CAN_GetTxPort(mailbox, drv->tx_id);
    /* port not found */
    if(tx_port == NULL) {
        tx_port = CAN_CreateTxPort(mailbox, drv);
    }
    /* set tx_port->mode to new mode(single/continous) */
    tx_port->mode = mode;    
}

/**
 *  @brief  根据tx_port找到对应的CAN口，并立即发送tx_port的内容
 */
HAL_StatusTypeDef CAN_StartTxByTxPort(CAN_TxPortTypeDef *tx_port)
{
    uint32_t mailbox;
    CAN_TxHeaderTypeDef frameheader;
    HAL_StatusTypeDef ret = HAL_ERROR; 
    
    if(tx_port != NULL) {
        portENTER_CRITICAL();
        
        frameheader.StdId = tx_port->tx_id;
        frameheader.IDE = CAN_ID_STD;
        frameheader.RTR = CAN_RTR_DATA;
        frameheader.DLC = 8;
        if((CAN_MailboxTypeDef*)tx_port->mailbox == &hcan1Mailbox)
            ret = HAL_CAN_AddTxMessage(&hcan1, &frameheader, &tx_port->tx_buff[0], &mailbox);
        else if((CAN_MailboxTypeDef*)tx_port->mailbox == &hcan2Mailbox)
            ret = HAL_CAN_AddTxMessage(&hcan2, &frameheader, &tx_port->tx_buff[0], &mailbox);
        
        tx_port->delta_tx_time = micros() - tx_port->last_tx_time;
        tx_port->last_tx_time = micros();
        /* once tx finished, disable the tx_port if it is set to single tx mode. */
        if(tx_port->mode == CAN_SINGLE_TX) {
            tx_port->state = DISABLE;
        }
        
        portEXIT_CRITICAL();
    }
    
    return ret;
}

/**
 *  @brief  根据drv_can找到对应的CAN口，并立即发送data的内容
 */
HAL_StatusTypeDef CAN_StartTxByDriver(struct drv_can *drv, uint8_t *data)
{
    uint32_t mailbox;
    CAN_TxHeaderTypeDef frameheader;
    HAL_StatusTypeDef ret = HAL_ERROR;
    
    frameheader.StdId = drv->tx_id;
    frameheader.IDE = CAN_ID_STD;
    frameheader.RTR = CAN_RTR_DATA;
    frameheader.DLC = 8;
    if(drv->id == DRV_CAN1)
        ret = HAL_CAN_AddTxMessage(&hcan1, &frameheader, data, &mailbox);
    else if(drv->id == DRV_CAN2)
        ret = HAL_CAN_AddTxMessage(&hcan2, &frameheader, data, &mailbox);
    
    drv->errno = (drv_errno_t)ret;
    if(drv->errno != DRV_OK)
        drv->err_cnt += 1;
    
    return ret;
}

/**
 *  @brief  邮箱轮询各个发送端口，若端口就绪则将其放入待发送端口列表。
 */
bool CAN_MailboxReadyForTx(CAN_MailboxTypeDef *mailbox)
{
    bool ret = false;
    CAN_TxPortTypeDef *tx_port = NULL;

    if(mailbox->auto_tx == ENABLE)
    {    
        if((mailbox != NULL) && (mailbox->tx_port_cnt > 0)) {
            tx_port = mailbox->tx_port;
            while(tx_port != NULL) {
                uint32_t cur_time = micros();
                /**
                 * 判断条件之所以是 >= (tx_port->tx_period-1)*1000，是因为send_task会在
                 * monitor_task进行任务通知之后才发送端口信息(同时更新端口的last_tx_time)
                 * ，这个任务切换的过程会有us级别的延迟。因此，下次monitor_task进行判断
                 * 的时候cur_time - tx_port->last_tx_time <= 1000us(=1ms)。如果判断条件
                 * 中的tx_period不减1，则需要多判断一次才满足条件，导致额外的1ms的延迟。
                 * 因此，这里可以通过修改判断条件为tx_period-1来保证发送间隔为tx_period,
                 * 避免额外的1ms的延迟。
                 * PS：这里所说的1ms指的是monitor_task的更新周期，默认1ms。
                 */
                if((cur_time - tx_port->last_tx_time) >= ((tx_port->tx_period-1)*1000)) {
                    if(tx_port->state == ENABLE) {
                        CAN_PushWaitTxPort(mailbox, tx_port);
                        ret = true;
                    }
                }
                tx_port = tx_port->next;
            }
        }
    }
    
    return ret;
}

/**
 *  @brief  邮箱按照待发送端口列表的顺序，将待发送端口的内容通过对应CAN口发送出去。
 */
void CAN_AutoTx(CAN_MailboxTypeDef *mailbox)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    
    if(mailbox != NULL) {
        while(mailbox->wait_tx_port_cnt) {
            tx_port = CAN_PopWaitTxPort(mailbox);
            if(HAL_ERROR == CAN_StartTxByTxPort(tx_port)) {
                mailbox->errno = CAN_MAILBOX_ERR;
                mailbox->err_cnt += 1;
            } else {
                mailbox->errno = CAN_MAILBOX_OK;
            }
        }
    }
}

/* Callback functions --------------------------------------------------------*/
/**
 *  @brief  重写 CAN RxFifo 中断接收函数
 *  @note   在stm32f4xx_hal_can.c中弱定义
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* CAN1 接收中断 */
    if(hcan->Instance == CAN1)
    {
        CAN_Rx_Callback(hcan);
        // HAL_CAN_DeactivateNotification
        // __HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);    // 暂停开启FIF00消息挂号中断，在消息处理任务中处理完成后再使能
    }else
    if(hcan->Instance == CAN2)
    {
        CAN_Rx_Callback(hcan);
    }
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *  @brief  [__WEAK] 需要在Protocol Layer中实现具体的 CAN1 处理协议
 */
__WEAK void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 *  @brief  [__WEAK] 需要在Protocol Layer中实现具体的 CAN2 处理协议
 */
__WEAK void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}
