/**
 *  @file       rp_monitor.c
 *  @author     @RobotPilots
 *  @version    v1.2
 *  @brief      rpMonitor task.
 *  @update     
 *              v1.0(26-January-2022)
 *              v1.1(1-Feburary-2022)
 *                  [*]1.修改发送机制，支持多帧同时发送
 *                  [*]2.修改接收机制，支持多帧解析(不支持断帧拼接)。加入忙碌状态
 *                       的判断，要求发送端的频率不能过快
 *              v1.2(26-Feburary-2022)
 *                  [+]1.增加RPM_REQ_NEW_TX_PERIOD指令及相关操作
 *                  [*]2.修改RPM_ERR为RPM_ERR_UNKNOWN(0xc0)与RPM_ERR_WRITE_ADDR(0xc1)
 */

/**
 *  @使用说明
 *      1.在freertos.c文件中添加以下代码
 *          ①包含头文件    
 *          // USER CODE BEGIN Includes
 *          #include "rp_monitor.h"
 *          // USER CODE END Includes
 *          
 *          ②定义线程句柄
 *          // USER CODE BEGIN Variables
 *          osThreadId rpMonitorTaskHandle;
 *          //USER CODE END Variables
 *
 *          ③在()中创建线程
 *          // USER CODE BEGIN RTOS_THREADS
 *          // add threads, ...
 *          osThreadDef(rpMonitorTask, StartrpMonitorTask, osPriorityNormal, 0, 256);
 *          rpMonitorTaskHandle = osThreadCreate(osThread(rpMonitorTask), NULL);
 *          // USER CODE END RTOS_THREADS
 *
 *      2.在Options for Target...(魔术棒)中的User标签页中，勾选After Build/Rebuild中的
 *          #Run #1，并在User Command中输入fromelf -a -o "$L@L.dtaddr" "#L"
 *
 *  @硬件配置
 *      默认使用串口1
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_monitor.h"

#include "string.h"
#include "drv_uart.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define RPM_UART            DRV_UART1

#define RPM_TX_PERIOD       5

#define RPM_SELF_ID         RPM_ID_CUSTOM_1
#define RPM_TARGET_ID       RPM_ID_SERVER_1

/* Private function prototypes -----------------------------------------------*/
static void rpMonitor_SubscribeFlash(void);
static rpm_frame_t* rpMonitor_ConfirmSubscribe(rpm_frame_t *rx_frame);
static rpm_frame_t* rpMonitor_ReadFlash(rpm_frame_t *rx_frame);
static rpm_frame_t* rpMonitor_WriteFlash(rpm_frame_t *rx_frame);
static void rpMonitor_NewTxPeriod(rpm_frame_t *rx_frame);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// local pointer to rpMonitor's members
static rpm_frame_t *rpm_tx_frame;
static rpm_frame_t *rpm_rx_frame;
static rpm_list_t *rpm_var_list;
// uartx tx buffer
static uint8_t rpm_tx_buf[RPM_BUFF_LEN];
// rpMonitor's driver
drv_uart_t rpm_drv = {
    .type = DRV_UART,
    .id = RPM_UART,
    .tx_buf = rpm_tx_buf,
    .tx_buf_len = RPM_BUFF_LEN,
    .tx_data_cnt = 0,
    .add_msg = USART_AddMsg,
    .start_tx = USART_StartTx,
    .start_tx_dma = USART_StartTxDma
};

/* Exported variables --------------------------------------------------------*/
// rpMonitor
rpMonitor_t rpMonitor = {
    .self_id = RPM_SELF_ID,
    .target_id = RPM_TARGET_ID,
    .var_list.cur_num = 0,
    .tx_period = RPM_TX_PERIOD,
    .offline_cnt = 0,
    .offline_cnt_max = 20
};

/* Private functions ---------------------------------------------------------*/
static void rpMonitor_Init(void)
{
    rpm_rx_frame = &rpMonitor.rx_frame;
    rpm_tx_frame = &rpMonitor.tx_frame;
    rpm_var_list = &rpMonitor.var_list;
}

static void rpMonitor_Rx(uint8_t *rxBuf)
{
    /* rpMonitor is busy for response now */
    if(rpMonitor.rx_update)
        return;
    
    memcpy((uint8_t*)rpm_rx_frame, (uint8_t*)rxBuf, 3);
    /* check sof */
    if(rpm_rx_frame->sof == 0xA5) 
    {
        // copy data_seg
        memcpy((uint8_t*)rpm_rx_frame->data_seg, (uint8_t*)(rxBuf + 3), rpm_rx_frame->data_len);
        // reset check_sum
        rpm_rx_frame->check_sum = rpm_rx_frame->sof + rpm_rx_frame->cmd_id + rpm_rx_frame->data_len;
        // calculate check_sum
        for(uint8_t i = 0; i < rpm_rx_frame->data_len; i++) {
            rpm_rx_frame->check_sum += rpm_rx_frame->data_seg[i];
        }
        /* check check_sum */
        if(rpm_rx_frame->check_sum == rxBuf[3 + rpm_rx_frame->data_len]) {
            switch(rpm_rx_frame->cmd_id)
            {
                case RPM_REQ_SUBSCRIBE: /* subscribe */
                    rpMonitor_ConfirmSubscribe(rpm_rx_frame);
                    break;
                
                case RPM_REQ_READ:      /* read */
                    rpMonitor_ReadFlash(rpm_rx_frame);
                    break;
                
                case RPM_REQ_WRITE:     /* write */
                    rpMonitor_WriteFlash(rpm_rx_frame);
                    break;
                
                case RPM_REQ_NEW_TX_PERIOD:/* new tx period */
                    rpMonitor_NewTxPeriod(rpm_rx_frame);
                    break;
                
                default:
                    break;
            }
            
            /* check next frame */
            if(rxBuf[3 + rpm_rx_frame->data_len + 1] == 0xA5) {
                rpMonitor_Rx(&rxBuf[3 + rpm_rx_frame->data_len + 1]);
            }
        }
    }
    
    rpMonitor.offline_cnt = 0;
    rpMonitor.rx_update = true;
}

static void rpMonitor_Tx(rpm_frame_t *tx_frame)
{
    uint8_t i;
    // copy tx_frame to rpm_tx_frame
    if(tx_frame != NULL)
        memcpy((uint8_t*)rpm_tx_frame, (uint8_t*)tx_frame, sizeof(rpm_frame_t));
    // copy rpm_tx_frame to txBuf
    rpm_drv.add_msg(&rpm_drv, (uint8_t*)rpm_tx_frame, 3+rpm_tx_frame->data_len);
    // calculate check_sum
    rpm_tx_frame->check_sum = rpm_tx_frame->sof + rpm_tx_frame->cmd_id + rpm_tx_frame->data_len;
    for(i = 0; i < rpm_tx_frame->data_len; i++) {
        rpm_tx_frame->check_sum += rpm_tx_frame->data_seg[i];
    }
    rpm_drv.add_msg(&rpm_drv, &rpm_tx_frame->check_sum, 1);
}

static void rpMonitor_SubscribeFlash(void)
{
    uint8_t idx = 0;
    uint32_t addr;
    uint8_t size;
    
    rpm_tx_frame->sof = 0xA5;
    rpm_tx_frame->cmd_id = RPM_RESP_SUBSCRIBE;
    for(uint8_t i = 0; i < rpm_var_list->cur_num; i++) {
        size = rpm_var_list->var[i].size;
        addr = rpm_var_list->var[i].addr;
        for(uint8_t j = 0; j < size; j++) {
            rpm_tx_frame->data_seg[idx++] = *(__IO uint8_t*)addr++;
        }
    }
    rpm_tx_frame->data_len = idx;
    // add rpm_tx_frame to rpm_drv
    rpMonitor_Tx(NULL);   
}

static rpm_frame_t* rpMonitor_ConfirmSubscribe(rpm_frame_t *rx_frame)
{
    static rpm_frame_t tx_frame;
    uint8_t idx = 0;
    uint32_t addr;
    uint8_t size;
    
    if(rx_frame != NULL) {
        // update rpm_var_list
        rpm_var_list->cur_num = rx_frame->data_len/5;
        for(uint8_t i = 0; i < rpm_var_list->cur_num; i++) {
            memcpy((uint32_t*)&rpm_var_list->var[i].addr, (uint8_t*)(rx_frame->data_seg + 5*i), 4);
            rpm_var_list->var[i].size = rx_frame->data_seg[5*i + 4];
        }
        // echo for confirm
        tx_frame.sof = 0xA5;
        tx_frame.cmd_id = RPM_RESP_SUBSCRIBE_OK;
        for(uint8_t i = 0; i < rpm_var_list->cur_num; i++) {
            size = rpm_var_list->var[i].size;
            addr = rpm_var_list->var[i].addr;
            memcpy((uint8_t*)(tx_frame.data_seg+5*i), (uint8_t*)&addr, 4);
            memcpy((uint8_t*)(tx_frame.data_seg+5*i+4), (uint8_t*)&size, 1);
            idx += 5;
        }
        tx_frame.data_len = idx;
    }
    
    return &tx_frame;   
}

static rpm_frame_t* rpMonitor_ReadFlash(rpm_frame_t *rx_frame)
{
    static rpm_frame_t tx_frame;
    uint8_t idx = 0;
    uint32_t addr;
    uint8_t size;

    if(rx_frame != NULL) {
        tx_frame.sof = 0xA5;
        tx_frame.cmd_id = RPM_RESP_READ_OK;
        // var_addr & var_size
        memcpy((uint8_t*)&addr, (uint8_t*)rx_frame->data_seg, 4);
        memcpy((uint8_t*)&size, (uint8_t*)(rx_frame->data_seg+4), 1);
        for(uint8_t i = 0; i < size; i++) {
            tx_frame.data_seg[idx++] = *(__IO uint8_t*)addr++;
        }
        tx_frame.data_len = idx;
    }
    return &tx_frame;  
}

static rpm_frame_t* rpMonitor_WriteFlash(rpm_frame_t *rx_frame)
{
    static rpm_frame_t tx_frame;
    uint8_t ret = false;
    uint8_t idx = 0;
    uint32_t addr;
    uint8_t size;
    uint64_t data = 0;
    
    if(rx_frame != NULL) {
        tx_frame.sof = 0xA5;
        // var's addr & var's size
        memcpy((uint8_t*)&addr, (uint8_t*)rx_frame->data_seg, 4);
        memcpy((uint8_t*)&size, (uint8_t*)rx_frame->data_seg+4, 1);
        // new var's value 
        memcpy((uint8_t*)&data, (uint8_t*)rx_frame->data_seg+5, size);
        /* overwrite var to flash */
        if(HAL_FLASH_Unlock() == HAL_OK) {
            if(HAL_FLASH_Program(size, addr, data) == HAL_OK) {
                if(HAL_FLASH_Lock() == HAL_OK) {
                    ret = true;
                }
            }
        }
        // response the overwrited value from flash
        for(uint8_t i = 0; i < size; i++) {
            tx_frame.data_seg[idx++] = *(__IO uint8_t*)addr++;
        }
        tx_frame.data_len = idx;
        /* check if write flash success */
        if(ret) {
            tx_frame.cmd_id = RPM_RESP_WRITE_OK;
        } else {
            tx_frame.cmd_id = RPM_ERR_WRITE_ADDR;
        }
    }
    
    return &tx_frame;
}

static void rpMonitor_NewTxPeriod(rpm_frame_t *rx_frame)
{
    uint16_t tx_period;
    
    if(rx_frame != NULL) {
        memcpy((uint8_t*)&tx_period, (uint8_t*)rx_frame->data_seg, 2);
        /* tx_period should be bigger than 1 and smaller than 10001(ms), [2,10000]*/
        if((tx_period > 1) && (tx_period <= 10000)) {
            rpMonitor.tx_period = tx_period;
        } else {
            rpMonitor.tx_period = RPM_TX_PERIOD;
        }
    }
}

static void rpMonitor_heart_beat(void)
{
    rpMonitor.offline_cnt++;
    if(rpMonitor.offline_cnt > rpMonitor.offline_cnt_max) {
        rpm_var_list->cur_num = 0;
    }
}

/* Exported functions --------------------------------------------------------*/
void USART1_rxDataHandler(uint8_t *rxBuf)
{
    rpMonitor_Rx(rxBuf);
}

void StartrpMonitorTask(void const * argument)
{
    rpMonitor_Init();
    
    for(;;)
    {
        if(rpm_var_list->cur_num)
            rpMonitor_SubscribeFlash();
        
        if(rpMonitor.rx_update) 
        {
            rpm_frame_t* tx_frame;
            
            tx_frame = rpMonitor_ConfirmSubscribe(NULL);
            if(tx_frame->cmd_id != 0) {
                rpMonitor_Tx(tx_frame);
                tx_frame->cmd_id = 0;
            }
            
            tx_frame = rpMonitor_ReadFlash(NULL);
            if(tx_frame->cmd_id != 0) {
                rpMonitor_Tx(tx_frame);
                tx_frame->cmd_id = 0;
            }
            
            tx_frame = rpMonitor_WriteFlash(NULL);
            if(tx_frame->cmd_id != 0) {
                rpMonitor_Tx(tx_frame);
                tx_frame->cmd_id = 0;
            }
            
            rpMonitor.rx_update = false;
        }
        
        //rpMonitor_heart_beat();
        if(rpm_drv.tx_data_cnt > 0)
            rpm_drv.start_tx_dma(&rpm_drv);
        
        osDelay(rpMonitor.tx_period);
    }
}
