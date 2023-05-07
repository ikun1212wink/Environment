/**
 *  @file       rp_config.h
 *  @author     @RobotPilots
 *  @version    v1.1
 *  @brief      RobotPilots Robots' Configuration.
 *  @update
 *              v1.0(9-September-2020)
 *              v1.1(7-November-2021)
 *                  1.优化设备类信息与结构体的变量定义，增加volatile/const关键字
 *                  2.将rp_config.h分成driver_config.h, device_config.h, user_config.h三个头文件    
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
    WFI;          
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
    CPSID   I
    BX      LR      
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
    CPSIE   I
    BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(uint32_t addr) 
{
    MSR MSP, r0             //set Main Stack value
    BX r14
}
