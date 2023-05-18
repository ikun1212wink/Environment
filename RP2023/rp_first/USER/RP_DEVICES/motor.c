/*
* EXAMPLE
*	RP MOTOR_BAG
*	2022.11.6 
* AUTHOR CCB HXZP
*
*ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ���Լ�initָ�뺯����ʼ��
*ʹ��pidǰҲ��Ҫ����pid��ʼ���������޷�ʹ��
*����ֱ��ʹ�����ú�������pid���� Ҳ����ֱ��ʹ�ÿ��ڶ��庯�������Լ�����ĺ���


ʹ�ò��裺
1��motor_list_e����б�����ӵ�����ƣ���ӵ��Ҫ����MOTOR_TESTǰ
2�����úõ����������ʽ�����͡�id�š���ʼ������
3������pid�����б���ʼ��pid
4�����巢�����飬�����ˣ


		��������Ϊ�Դ�pid���Ƽ��㣬�Զ���������pid�飬Ҫ�Լ���ʼpid
		motor_pid_t            speed;
	
		motor_pid_t            angle;	
		motor_pid_t            angle_in;		
	
		motor_pid_t            position;
		motor_pid_t            position_in;	
		
		targetֵ��λ�ֱ�Ϊ
		8192 
		8192(�����ڼ����Զ�ת360��) 
		rpm 

	float (*c_posit)(struct motor_class_t *motor,float target);
	float (*c_angle)(struct motor_class_t *motor,float target);	
	float (*c_step)(struct motor_class_t *motor,float target,char step);
	float (*c_speed)(struct motor_class_t *motor,float target);	
	

		�û��Զ���pid
		err_cal_modeΪerr����ʽ�� 0��1��2��һȦ����Ȧ���ķ�֮һȦ
		meas ʹ��360���� ����Ƕ�8192���ﲻ���Զ�ת360

	float (*c_pid2)(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, float tar, char err_cal_mode);	
	float (*c_pid1)(motor_pid_t *out, float meas1, float tar);	
	
���ڷ��ͣ�
����ṹ�����Դ��˷��ͺ�����
����Ҫ�Լ����巢�͵����飬
���Ҵ���ͬһ����ĵ�����ݻ�ͳһ���ͣ����ֻҪ����һ�η����ĸ�������ݣ�������ŵ��id
���������ݶ�Ϊ0ʱ���ᷢ�ͣ����ͽ��������ݰ������㣬�����ظ�����


*/
#include "RP_CONFIG.h"
#include "motor.h"


/*----------------------------------------�û��ӿ�begin----------------------------------------*/


/*
* ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ
*/
motor_t motor[MOTOR_LIST] =
{
	[GIMB_Y] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id      = 0x205,

		.init = &motor_class_init,
	},
	[GIMB_R] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id      = 0x205,
		
		.init = &motor_class_init,
	},
		
	[GIMB_P] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id      = 0x206,
		
		.init = &motor_class_init,
	},
	
	[MOTOR_TEST] = {
	
		.id.drive_type = (motor_drive_e)RM_MOTOR_CAN_TYPR_TEST,
		.id.motor_type = (motor_type_e)RM_MOTOR_TYPE_TEST,
		.id.rx_id      = (motor_id_e)RM_MOTOR_CAN_ID_TEST,
		
		.init = &motor_class_init,
	}
};

/*--------------------------------------------------------------------------*/

/*pid�����б�*/

/*
	��Ҫ����һ��������Ϊ�����������ʹ�ýṹ������Ϊ�����ʼ��չ��Ϊһά�Ŀ������Ƚϼ�Լ������Ҫ��ס���ǵ�λ�úͺ���
	float	  kp;
	float 	ki;
	float 	kd;
	
	float  blind_err;	
	float  integral_max;	
	float  iout_max;
	float  out_max;
*/
float  test_speed_pid_param[7] = {5,0,0,0,0,15000,10000};
float  test_posit_pid_param[7] = {1,0,0,0,0,15000,20000};
float  test_posit_in_pid_param[7] = {2,0.1,0,0,0,5000,10000};

float  test_angle_pid_param[7] = {20,0,0,0,0,15000,20000};
float  test_angle_in_pid_param[7] = {10,1,50,0,0,8000,16000};

float yaw_imu_out_pid_param[7] = {15,0,0,0,0,15000,20000};
float yaw_imu_inn_pid_param[7] = {100,0.3,0,0,15000,20000,20000};
float pit_imu_out_pid_param[7] = {20,0,0,0,0,15000,20000};
float pit_imu_inn_pid_param[7] = {300,0.2,0,0,15000,20000,20000};

void RM_MotorInit(void)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].init(&motor[i]);
	}
	/*--------------------------------------------------------------*/
	/*����ٶ�pid��ʼ��*/
	
	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.speed,test_speed_pid_param);
	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.position,test_posit_pid_param);
	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.position_in,test_posit_in_pid_param);

	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.step,test_angle_pid_param);
	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.step_in,test_angle_in_pid_param);	
	
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle,   yaw_imu_out_pid_param);
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle_in,yaw_imu_inn_pid_param);	
	
	motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle,   pit_imu_out_pid_param);
	motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle_in,pit_imu_inn_pid_param);	
}


/*
	������Դ���
*/
float targetTest = 300;
int16_t SendBuffTest[4];

void RM_MotorControl_Test(void)
{
  	SendBuffTest[motor[MOTOR_TEST].id.buff_p] = motor[MOTOR_TEST].c_step(&motor[MOTOR_TEST],0,3);

	
//	  SendBuffTest[motor[MOTOR_TEST].id.buff_p] = motor[MOTOR_TEST].c_posit(&motor[MOTOR_TEST],8192*14);
	
//		SendBuffTest[motor[MOTOR_TEST].id.buff_p] = motor[MOTOR_TEST].c_speed(&motor[MOTOR_TEST],targetTest);
	  //�������ݵķ���
		motor[MOTOR_TEST].tx(&motor[MOTOR_TEST],SendBuffTest);
	

}



/*----------------------------------------�û��ӿ�end----------------------------------------*/






/*----------------------------------------�ײ��Զ�����----------------------------------------*/










char RM_MotorHeartBeat(void)
{
	char offline_cnt = 0;
	
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].heartbeat(&motor[i]);

		if(motor[i].state.work_state == M_OFFLINE)offline_cnt++;
	}
	
	//ȥ�����Ե���ĸ���
	if(motor[MOTOR_LIST-1].state.work_state == M_OFFLINE && offline_cnt)offline_cnt -= 1;
		
	return offline_cnt;
	
}


/*

����������ݽӿ�

*/

void RM_MotorCAN1(uint32_t canId, uint8_t *rxBuf)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].rx(&motor[i],rxBuf,canId,M_CAN1);
	}
	
}

void RM_MotorCAN2(uint32_t canId, uint8_t *rxBuf)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].rx(&motor[i],rxBuf,canId,M_CAN2);
	}
}







