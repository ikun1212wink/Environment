/**
 *  @file       rc_sensor.c
 *  @author     @RobotPilots
 *  @version    v1.1
 *  @brief      Device Rc.
 *  @update
 *              v1.0(9-September-2020)
 *              v1.1(9-Feburary-2022)
 *                  [*]1.将rc_sensor_init()移动到该文件
 *                  [*]2.修改if_channel_reset()和reset()函数为成员函数
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"

#include "rp_math.h"
#include "rc_protocol.h"

extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_init(rc_sensor_t *rc_sen);
static void rc_sensor_check(rc_sensor_t *rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);
static bool rc_sensor_if_channel_reset(rc_sensor_t *rc_sen);
static void rc_sensor_reset(rc_sensor_t *rc_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 遥控器驱动
drv_uart_t          rc_sensor_driver = {
    .id = DRV_UART2,
};

// 遥控器信息
rc_sensor_info_t    rc_sensor_info = {
    .offline_max_cnt = 60,
};

// 遥控器传感器
rc_sensor_t         rc_sensor = {
    .info = &rc_sensor_info,
    .init = rc_sensor_init,
    .update = rc_sensor_update,
    .check = rc_sensor_check,
    .heart_beat = rc_sensor_heart_beat,
    .work_state = DEV_OFFLINE,
    .id = DEV_ID_RC,
    .if_channel_reset = rc_sensor_if_channel_reset,
    .reset = rc_sensor_reset
};

/* Private functions ---------------------------------------------------------*/
static void rc_sensor_init(rc_sensor_t *rc_sen)
{
    // 初始化为离线状态
    rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
    rc_sen->work_state = DEV_OFFLINE;
    
    if(rc_sen->id == DEV_ID_RC)
        rc_sen->errno = NONE_ERR;
    else
        rc_sen->errno = DEV_ID_ERR;
}

/**
 *  @brief  遥控器数据检查
 */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;
    
    if(abs(rc_info->ch0) > 660 ||
       abs(rc_info->ch1) > 660 ||
       abs(rc_info->ch2) > 660 ||
       abs(rc_info->ch3) > 660)
    {
        rc_sen->errno = DEV_DATA_ERR;
        rc_info->ch0 = 0;
        rc_info->ch1 = 0;
        rc_info->ch2 = 0;
        rc_info->ch3 = 0;        
        rc_info->s1 = RC_SW_MID;
        rc_info->s2 = RC_SW_MID;
        rc_info->thumbwheel = 0;
    }
    else
    {
        rc_sen->errno = NONE_ERR;
    }
}

/**
 *  @brief  遥控器心跳包
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;

    rc_info->offline_cnt++;
    if(rc_info->offline_cnt > rc_info->offline_max_cnt) {
        rc_info->offline_cnt = rc_info->offline_max_cnt;
        rc_sen->work_state = DEV_OFFLINE;
    } 
    else {
        /* 离线->在线 */
        if(rc_sen->work_state == DEV_OFFLINE)
            rc_sen->work_state = DEV_ONLINE;
    }
}

/**
 *  @brief  遥控器判断通道是否归位
 */
static bool rc_sensor_if_channel_reset(rc_sensor_t *rc_sen)
{
    bool ifChannelReset = false;
    rc_sensor_info_t *rc_info = rc_sen->info;
    
    if(  (DeathZoom(rc_info->ch0, 0, 50) == 0) && 
         (DeathZoom(rc_info->ch1, 0, 50) == 0) && 
         (DeathZoom(rc_info->ch2, 0, 50) == 0) && 
         (DeathZoom(rc_info->ch3, 0, 50) == 0)   )    
    {
        ifChannelReset = true;
    }
    return ifChannelReset;
}

/**
 *  @brief  遥控器强制复位遥控值
 */
static void rc_sensor_reset(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;
    
    // 通道值强行设置成中间值(不拨动摇杆的状态)
    rc_info->ch0 = 0;
    rc_info->ch1 = 0;
    rc_info->ch2 = 0;
    rc_info->ch3 = 0;
    // 左右开关选择强行设置成中间值状态
    rc_info->s1 = RC_SW_MID;
    rc_info->s2 = RC_SW_MID;
    // 鼠标
    rc_info->mouse_vx = 0;
    rc_info->mouse_vy = 0;
    rc_info->mouse_vz = 0;
    rc_info->mouse_btn_l = 0;
    rc_info->mouse_btn_r = 0;
    // 键盘
    rc_info->key_v = 0;
    // 左拨轮
    rc_info->thumbwheel = 0;    
}

/* Exported functions --------------------------------------------------------*/
