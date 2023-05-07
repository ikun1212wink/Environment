/**
 *  @file       pid.h
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Pid Algorithm.
 *  @update     
 *              v1.0(11-September-2020)
 */
#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void pid_val_init(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
void cascade_pid_ctrl(pid_ctrl_t *pid);

#endif
