/**
 *  @file       rp_monitor.h
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
#ifndef __RP_MONITOR_H
#define __RP_MONITOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define RPM_VAR_MAX_NUM     8
#define RPM_BUFF_LEN        (4 + (RPM_VAR_MAX_NUM*5)*2)

/* Exported types ------------------------------------------------------------*/
/**
 *  @brief  rpm_id determines the role of the board.
 */
enum rpm_id {
    // 0x0x std id
    RPM_ID_HERO = 0x01,
    RPM_ID_ENGINEERING = 0x02,
    RPM_ID_INFANTRY_3 = 0x03,
    RPM_ID_INFANTRY_4 = 0x04,
    RPM_ID_INFANTRY_5 = 0x05,
    RPM_ID_SENTRY = 0x06,
    RPM_ID_AERIAL = 0x07,
    RPM_ID_MISSLE = 0x08,
    RPM_ID_OUTPOST = 0x09,
    RPM_ID_BASE = 0x0a,
    
    // 0x1x
    // reserved
    
    // 0x2x custom
    RPM_ID_CUSTOM_1 = 0x21,
    RPM_ID_CUSTOM_2 = 0x22,
    RPM_ID_CUSTOM_3 = 0x23,
    RPM_ID_CUSTOM_4 = 0x24,
    RPM_ID_CUSTOM_5 = 0x25,
    RPM_ID_CUSTOM_6 = 0x26,
    RPM_ID_CUSTOM_7 = 0x27,
    RPM_ID_CUSTOM_8 = 0x28,
    
    // 0x4x
    // reserved
    
    // 0x8x
    RPM_ID_SERVER_1 = 0x81,
    RPM_ID_SERVER_2 = 0x82,
};

/**
 *  @brief  rpm_cmd consists of three kinds of cmd:
 *                  + request
 *                  + response
 *                  + err
 */
enum rpm_cmd {
    // requset cmd
    RPM_REQ_SUBSCRIBE = 0x01,
    RPM_REQ_READ = 0x02,
    RPM_REQ_WRITE = 0x03,
    RPM_REQ_NEW_TX_PERIOD = 0x04,
    // correct response cmd
    RPM_RESP_SUBSCRIBE = 0x80,
    RPM_RESP_SUBSCRIBE_OK = 0x81,
    RPM_RESP_READ_OK = 0x82,
    RPM_RESP_WRITE_OK = 0x83,
    //RPM_RESP_NEW_TX_PERIOD_OK = 0x84,
    // err response cmd
    RPM_ERR_UNKNOWN = 0xc0,
    RPM_ERR_WRITE_ADDR = 0xc1,
};

/**
 *  @brief  rpm_frame
 */
typedef __packed struct {
    uint8_t sof;        // 同步头
    uint8_t cmd_id;     // 请求/响应/错误代码
    uint8_t data_len;   // 数据段(data_seg)长度
    uint8_t data_seg[RPM_VAR_MAX_NUM*5];   // 数据段数据内容
    uint8_t check_sum;  // 校验和
} rpm_frame_t;

/**
 *  @brief  rpm_var
 */
typedef struct {
    uint32_t addr;
    uint8_t  size;
} rpm_var_t;

/**
 *  @brief  rpm_var_list
 */
typedef struct {
    rpm_var_t   var[RPM_VAR_MAX_NUM];
    uint8_t     cur_num;
} rpm_list_t;

/**
 *  @brief  rpMonitor
 */
typedef struct {
    enum rpm_id         self_id;
    enum rpm_id         target_id;
    rpm_list_t          var_list;
    rpm_frame_t         rx_frame;
    rpm_frame_t         tx_frame;
    uint16_t            tx_period;    
    volatile uint8_t    rx_update;
    uint16_t            offline_cnt;
    uint16_t            offline_cnt_max;
} rpMonitor_t;

/* Exported functions --------------------------------------------------------*/
void StartrpMonitorTask(void const * argument);

#endif
