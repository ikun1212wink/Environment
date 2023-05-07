/**
 *  @file       device.c
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Devices' Manager.
 *  @update     
 *              v1.0(15-September-2020)
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
    .rc_sen = &rc_sensor,
    .imu_sen = &imu_sensor,
    .chas_mtr[CHAS_LF] = &chassis_motor[CHAS_LF],
    .chas_mtr[CHAS_RF] = &chassis_motor[CHAS_RF],
    .chas_mtr[CHAS_LB] = &chassis_motor[CHAS_LB],
    .chas_mtr[CHAS_RB] = &chassis_motor[CHAS_RB],
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEV_Init(void)
{
    dev_list.rc_sen->init(dev_list.rc_sen);
    dev_list.imu_sen->init(dev_list.imu_sen);
    dev_list.chas_mtr[CHAS_LF]->init(dev_list.chas_mtr[CHAS_LF]);
    dev_list.chas_mtr[CHAS_RF]->init(dev_list.chas_mtr[CHAS_RF]);
    dev_list.chas_mtr[CHAS_LB]->init(dev_list.chas_mtr[CHAS_LB]);
    dev_list.chas_mtr[CHAS_RB]->init(dev_list.chas_mtr[CHAS_RB]);
}
