/**
 * @file        system_task.c
 * @author      @RobotPilots
 * @version     v1.2
 * @brief       Decision Center.
 *  @update     
 *              v1.0(27-October-2020)
 *              v1.1(9-Feburary-2022)
 *                  [+]1.新增模块rc
 *                  [*]2.修改系统任务处理，增加重联等待
 *              v1.2(6-March-2022)
 *                  [+]1.system_task任务开始时调用rc.init()
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"

#include "cmsis_os.h"
#include "rc.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
flag_t flag = {
    .gimbal = {
        .reset_start = false,
        .reset_ok = false,
    },
};

system_t sys = {
    .remote_mode = RC,
    .state = SYS_STATE_RCLOST,
    .mode = SYS_MODE_NORMAL,
};

/* Private functions ---------------------------------------------------------*/
/**
 *  @brief  根据遥控器切换控制方式
 */
static void system_ctrl_mode_switch(void)
{
    
}

/**
 *  @brief  根据遥控器切换系统行为
 */
static void system_mode_act_switch(void)
{
//    if(rc.info->W.flip == PRESS_TO_RELEASE) {
//        
//    }
}

static void system_state_machine(void)
{
    // 控制方式切换
    system_ctrl_mode_switch();
    // 系统模式切换(键盘模式下才允许切换)
    if(sys.remote_mode == KEY)
        system_mode_act_switch();
}

/* Exported functions --------------------------------------------------------*/
/**
 *  @brief  系统决策任务
 */
void StartSystemTask(void const * argument)
{   
    static uint8_t reconnect_cnt = 0;
    
    rc.init();
    for(;;)
    {
        portENTER_CRITICAL();
        
        /* 遥控失联 */
        if(rc.dev->work_state == DEV_OFFLINE) {
            // 重置rc传感器
            rc.dev->reset(rc.dev);
            // 重置rc模块
            rc.reset();
            // 重联计数清零
            reconnect_cnt = 0;
            // 系统：遥控失联
            sys.state = SYS_STATE_RCLOST;
        }
        /* 遥控在线 */
        else {
            // 更新rc模块
            rc.update();  
            /* 遥控出错 */
            if(rc.dev->errno != NONE_ERR) {
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            } 
            /* 遥控正常 */
            else {
                /* 失联恢复 */
                if(sys.state == SYS_STATE_RCLOST) {
                    if(reconnect_cnt < 2) {
                        reconnect_cnt++;
                    } else {
                        // 可在此处同步云台复位标志位
                        flag.gimbal.reset_start = true;
                        flag.gimbal.reset_ok = false;
                        // 系统：参数复位
                        sys.remote_mode = RC;
                        sys.state = SYS_STATE_NORMAL;
                        sys.mode = SYS_MODE_NORMAL;
                    }
                }
                /* 系统正常 */
                else if(sys.state == SYS_STATE_NORMAL) {
                    // 可在此处等待云台复位后才允许切换状态
                    if(flag.gimbal.reset_ok == true)
                        system_state_machine();
                }
            }
        }
        
        portEXIT_CRITICAL();
        
        osDelay(2);
    }
}
