/**
 *  @file       device.h
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Devices' Manager.
 *  @update     
 *              v1.0(15-September-2020)
 */
#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"      // v1.1
#include "imu_sensor.h"     // v1.0
#include "chassis_motor.h"  // v1.2

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
    rc_sensor_t         *rc_sen;
    imu_sensor_t        *imu_sen;
    chassis_motor_t     *chas_mtr[CHAS_MOTOR_CNT];
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
