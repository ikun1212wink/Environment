#ifndef __RM_MOTOR_DEF_H
#define __RM_MOTOR_DEF_H

#include "stm32f4xx_hal.h"

////RM2006 RM3508
//#define RM_F_ID 0x200
//#define RM_B_ID 0x1FF
////GM6020
//#define GM_F_ID 0x1FF
//#define GM_B_ID 0x2FF

typedef enum{

  //RM2006 RM3508
  RM_F_ID = 0x200,
  RM_B_ID = 0x1FF,
  //GM6020
  GM_F_ID = 0x1FF,
  GM_B_ID = 0x2FF,

}motor_id_e;


typedef enum 
{
	M_OFFLINE = 0,	
	
	M_ONLINE,

	M_TYPE_ERR,
	M_ID_ERR,
	M_INIT_ERR,	
	M_DATA_ERR,
	
}motor_state_e;

typedef enum{

	M_DEINIT = 0,
	M_INIT,

}motor_init_e;//��ʼ��ö��

typedef enum{
	M_CAN1 = 1,
	M_CAN2,
	
	M_PWM,
	
}motor_drive_e;//�������ͨ�ŷ�ʽ

typedef enum{
	
	GM6020 = 1,
	RM3508,
	RM2006,
	
	GM3510,
	RM3510,

	
}motor_type_e;//�������

typedef enum {
	MOTOR_F,
	MOTOR_B
} motor_dir_t;//�������


/*-----------------------------------------------------------------*/

typedef struct 
{
	int16_t		angle;						//0~8191 //������ʼ����Ϣ
	int16_t 	speed;						//RPM    //����ٶ�
	int16_t 	current;					//       //ʵ��ת��
	int16_t		temperature;			//��C    //�¶�
	int16_t 	torque;		
	
	int16_t		angle_prev;	
	int16_t		angle_offset;			//       //ƫִ �Զ���Ƕȷ�Χ
	int32_t		angle_sum;				//-2147683647~2147683647 �ϵ翪ʼ�����ڵĽǶȺ�

}motor_rx_info_t;

typedef struct 
{
	int16_t     mec_mid;  //��е��ֵ
	motor_dir_t dir;	    //����
	
}motor_mec_info_t;//��е��Ϣ

typedef struct
{
	uint32_t tx_id;   //����id
	uint32_t rx_id;   //����id
	uint32_t buff_p;  //����/���� ����λ��
	
	motor_drive_e drive_type; 
	motor_type_e  motor_type;
	
}motor_id_info_t;//�����ID��Ϣ

typedef struct
{
	motor_init_e  init_flag;	
	
	uint8_t       offline_cnt_max;
	uint8_t       offline_cnt;

	motor_state_e work_state;	
	
}motor_state_info_t;//�����ID��Ϣ
/*-----------------------------------------------------------------*/

typedef struct 
{
	float	  target;
	float	  measure;
	float 	err;
	float 	last_err;
	
	float	  integral;
	
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	
	motor_init_e init_flag;
	
}motor_pid_info_t; //pid��Ϣ

typedef struct 
{
	/* ���� */
	float	  kp;
	float 	ki;
	float 	kd;
	
	float   blind_err;	
	float 	integral_max;	
	float   iout_max;
	float 	out_max;
	
}motor_pid_set_t; //pid����

typedef struct motor_pid
{
	motor_pid_info_t info;
	motor_pid_set_t  set;
	
}motor_pid_t; //pid��С��λ


typedef struct
{
	motor_pid_t   speed;
	
	motor_pid_t   angle;	
	motor_pid_t   angle_in;		
	
	motor_pid_t   step;         
	motor_pid_t   step_in;
	
	motor_pid_t   position;
	motor_pid_t   position_in;	
	
	motor_pid_t   user_define;
	
}motor_pid_all_t; //pid�ܻ�


#endif


