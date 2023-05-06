#ifndef __MOTOR_H
#define __MOTOR_H


#include "DRIVERS.h"
#include "rm_motor.h"


typedef enum
{
	GIMB_Y,
	GIMB_P,
  GIMB_R,
	MOTOR_TEST,
	
	MOTOR_LIST,
	
}motor_list_e;


void RM_MotorCAN1(uint32_t canId, uint8_t *rxBuf);
void RM_MotorCAN2(uint32_t canId, uint8_t *rxBuf);

void RM_MotorInit(void);
char RM_MotorHeartBeat(void);
void RM_MotorControl_Test(void);

extern motor_t motor[MOTOR_LIST];

#endif


