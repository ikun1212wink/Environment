/*
*	RP MOTOR_BAG
*	2022.11.6 
* AUTHOR CCB HXZP
*


*ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ,�Լ���ʼ������ָ��motor_class_init
*ʹ��pidǰҲ��Ҫ����pid��ʼ���������޷�ʹ��
*����ֱ��ʹ�����ú�������pid���� Ҳ����ֱ��ʹ�ÿ��ڶ��庯�������Լ�����ĺ���

*/

#include "rm_motor.h"
#include "can_drv.h"
#include "string.h"


/**
 *	@brief	���������Ϣ
 */
uint8_t rm_motor_tx(struct motor_class_t *motor, int16_t *buff)
{
	uint8_t res,cnt = 0;
	
	if(motor == NULL || buff == NULL)
	{
		return 0;
	}	

	for(char i = 0;i < 4;i++)
	{
		if(buff[i] == 0)cnt++;
		if(cnt == 4)return 0;
	}
	
	if(motor->id.drive_type == M_CAN1){
	
		
		res = CAN1_SendData(motor->id.tx_id,buff);
	}
	else if(motor->id.drive_type == M_CAN2){
	
	
		res = CAN2_SendData(motor->id.tx_id,buff);
	}	
	
	for(char i = 0;i < 4;i++)
	{
		buff[i] = 0;
	}
	
	return res;
}

/**
 *	@brief	���������Ϣ
 */
uint8_t rm_motor_rx(struct motor_class_t *motor, uint8_t *buff,uint32_t id,motor_drive_e driver)
{
	uint8_t res;
	
	if(motor == NULL || buff == NULL)
	{
		return 0;
	}
	
	if(motor->state.init_flag == M_DEINIT)
	{
		return 0;	
	}
	
	if(motor->id.rx_id != id)
	{
		return 0;	
	}
	
	if(motor->id.drive_type != driver)
	{
		return 0;	
	}
	
	get_rm_info(motor,buff);

	return res;
}


















/**
 *	@brief	���pid��ʼ��
 */
void motor_class_pid_init(struct motor_pid *pid, float *buff)
{
	if(pid == NULL)
	{
		return;
	}
	
	memset(&pid->info,0,sizeof(pid->info));
	
	if(buff != NULL){
		
		memcpy(&pid->set,buff,sizeof(pid->set));

	}
	else if(buff == NULL){
		
		memset(&pid->set,0,sizeof(pid->set));

	}
	
	pid->info.init_flag = M_INIT;
	
}	

/**
 *	@brief	�����ʼ��
 */
void motor_class_init(struct motor_class_t *motor)
{
	if(motor == NULL)
	{
		return;
	}
	
	motor->state.work_state = M_OFFLINE;
	
	motor->state.offline_cnt = 100;
	motor->state.offline_cnt_max = 100;	
	

	if(motor->id.drive_type == M_CAN1 || motor->id.drive_type == M_CAN2)
	{
		get_rm_can_drvie(motor);
	}
	
	motor->pid_init     = motor_class_pid_init;
	motor->heartbeat    = motor_class_heartbeat;	
	
	motor->c_stuck_flag = motor_class_stucking_flag;
	motor->c_offset     = motor_offset;
	motor->c_judge_dir  = motor_judge_dir;
	
	motor->c_speed = motor_pid_speed;
	motor->c_angle = motor_pid_angle;
	motor->c_step  = motor_pid_step;
	motor->c_posit = motor_pid_position;	
	
	motor->c_pid1 = motor_pid_single;
	motor->c_pid2 = motor_pid_double;		
	
	motor->state.init_flag = M_INIT;
}

/**
 *	@brief	�������
 */
void motor_class_heartbeat(struct motor_class_t *motor)
{
	if(motor == NULL)
	{
		return;
	}

	if(motor->state.init_flag == M_DEINIT)
	{
		return;	
	}
	
	motor->state.offline_cnt++;
	
	if(motor->state.offline_cnt > motor->state.offline_cnt_max) 
	{
		motor->state.offline_cnt = motor->state.offline_cnt_max;
		motor->state.work_state = M_OFFLINE;
	}
	else 
	{
		if(motor->state.work_state == M_OFFLINE)
			motor->state.work_state = M_ONLINE;
	}
}

/**
*	@brief	��ת�жϣ����Ǳ�Ҫ������������һ����ס torque_limit:Ť����ֵ return:1Ϊ�� 0Ϊ��
 */
uint8_t motor_class_stucking_flag(struct motor_class_t *motor,uint16_t torque_limit)
{
	uint8_t res = 0;
	
	if(motor->state.init_flag == M_DEINIT || motor->state.work_state == M_OFFLINE)return 0;	
	
	if(m_abs(motor->rx_info.torque) > torque_limit && m_abs(motor->rx_info.speed) < 50){
	
		res = 1;
	
	}
	else{
	
		res = 0;
	
	}
	
	return res;
	
}

/*-----------------------------------------------------------------
*���ݴ���
-----------------------------------------------------------------*/
/**
 *	@brief	���ڴ��� tar��Դ���� cycle:���ݷ�Χ
 */
float motor_cycle(float tar,float cycle)
{
	if(tar < 0)          tar += cycle;
	else if(tar >=cycle) tar -= cycle;
	
	return tar;
}
/**
 *	@brief	����Ȧ���� angle��Դ���� cycle:���ݷ�Χ
 */
float motor_half_cycle(float angle,float max)
{
	if(m_abs(angle) > (max/2))
	{	
		if(angle >= 0)
			angle += -max;		
		else
			angle +=  max;
	}
	return angle;
}


/**
 *	@brief	����������  limit:������ݷ�Χ
 */
void motor_judge_dir(struct motor_class_t *motor,uint16_t range)
{
	int16_t angle = 0;
	
	if(motor->rx_info.angle < motor->mec_info.mec_mid)
		 angle = motor->rx_info.angle - motor->mec_info.mec_mid + range;
	else
	   angle = motor->rx_info.angle - motor->mec_info.mec_mid;

	if(m_abs(angle - range/2) < range/4)
	{
		motor->mec_info.dir = MOTOR_B;
	}
	else 
	{
		motor->mec_info.dir = MOTOR_F;
	}	
}
/**
 *	@brief	�Ե���Ƕ���ƫ�� ����yaw��������ԭʼ��ǰΪ1777���޸ĺ�ǰΪ0 
 *            range:������ݷ�Χ
 */
void motor_offset(struct motor_class_t *motor, uint16_t range)
{
	motor_rx_info_t *info = &motor->rx_info;
	
	int16_t angle = 0;
	
	if(info->angle < motor->mec_info.mec_mid)
		 angle = info->angle - motor->mec_info.mec_mid + range;
	else
	   angle = info->angle - motor->mec_info.mec_mid;
	
  angle = -angle + range + range/4;
	
	if(angle > range)angle = angle - range;

	angle = motor_cycle(angle - range/4,range);
	
	info->angle_offset = angle;			
}


/*-----------------------------------------------------------------
*���pid
-----------------------------------------------------------------*/

/**
 *	@brief	��ȡ��� tar - mea
 */
float motor_pid_err(motor_pid_t *pid,float measure)
{
	motor_pid_info_t *pid_info = &pid->info;
	
	if(pid->info.init_flag == M_DEINIT)
	{
		return 0;
	}
	
	pid_info->measure = measure;
	pid_info->err = pid_info->target - pid_info->measure;
	return pid_info->err;

}

/**
 *	@brief	pid���� ������err����
 */
float motor_pid_cal(motor_pid_t *pid)
{
	
	if(pid->info.init_flag == M_DEINIT)
	{
		return 0;
	}
	
	motor_pid_info_t *pid_info = &pid->info;
	motor_pid_set_t  *pid_set = &pid->set;	
	
	//�������ֵ(��Ҫ���������м������)
	//pid->err = err;
	if(m_abs(pid_info->err)<=(pid_set->blind_err))
		pid_info->err = 0;
	
	//����
	pid_info->integral += pid_info->err;
	pid_info->integral = m_constrain(pid_info->integral, -pid_set->integral_max, +pid_set->integral_max);
	
	//pid ��������
	pid_info->pout = pid_set->kp * pid_info->err;
	pid_info->iout = pid_set->ki * pid_info->integral;
	pid_info->dout = pid_set->kd * (pid_info->err - pid_info->last_err);
	
	pid_info->iout = m_constrain(pid_info->iout, -pid_set->iout_max, pid_set->iout_max);	
	//�ۼ�pid���ֵ
	pid_info->out = pid_info->pout + pid_info->iout + pid_info->dout;
	pid_info->out = m_constrain(pid_info->out, -pid_set->out_max, pid_set->out_max);

	pid_info->last_err = pid_info->err;
	
	return pid_info->out;
}

/**
 *	@brief	pid�ܿ��� �������⻷ �ڻ� �⻷�۲�ֵ �ڻ��۲�ֵ err����ʽ
 *          err_cal_mode��err����ʽ ��Ȧ�����ķ�֮һȦ 0��1��2 �ٶȻ�ʹ��0 yaw��ʹ��1
 *          ���ڻ�ΪNULL��ֻ�����⻷���⻷����ΪNULL
 *
 *  @return ���ؼ�����
 */
float motor_pid_ctrl(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, char err_cal_mode)
{
	if(out == NULL)return 0;
	
	if(inn == NULL)
	{
		motor_pid_err(out , meas1);	
//		switch(err_cal_mode)
//		{
//			case 0:			
//				break;
//			
//			case 1:
//				while(m_abs(out->info.err) >= 360) 
//					out->info.err = motor_cycle(out->info.err,360);
//				
//				out->info.err = motor_half_cycle(out->info.err, 360);
//				break;				
//			
//			case 2:
//				while(m_abs(out->info.err) >= 360) 
//					out->info.err = motor_cycle(out->info.err,360);
//				
//				out->info.err = motor_half_cycle(out->info.err, 360);
//				out->info.err = motor_half_cycle(out->info.err, 180);
//				break;			
//		}

		if(err_cal_mode > 0){
			
			while(m_abs(out->info.err) >= 360){
			
				out->info.err = motor_cycle(out->info.err,360);		
			}
			
			for(char i = 1;i < err_cal_mode+1; i++){
			
				out->info.err = motor_half_cycle(out->info.err, 360/err_cal_mode);
			}
		}

		motor_pid_cal(out);
		
		return out->info.out;	
	}
	else 
	{
		/*--�⻷����--*/
		motor_pid_err(out , meas1);	
		
//		switch(err_cal_mode)
//		{
//			case 0:			
//				break;
//			
//			case 1:
//				while(m_abs(out->info.err) >= 360) 
//					out->info.err = motor_cycle(out->info.err,360);
//				
//				out->info.err = motor_half_cycle(out->info.err, 360);
//				break;				
//			
//			case 2:
//				while(m_abs(out->info.err) >= 360) 
//					out->info.err = motor_cycle(out->info.err,360);
//				
//				out->info.err = motor_half_cycle(out->info.err, 360);
//				out->info.err = motor_half_cycle(out->info.err, 180);
//				break;		
//		}
		if(err_cal_mode > 0){
			
			while(m_abs(out->info.err) >= 360){
			
				out->info.err = motor_cycle(out->info.err,360);		
			}
			
			for(char i = 1;i < err_cal_mode+1; i++){
			
				out->info.err = motor_half_cycle(out->info.err, 360/err_cal_mode);
			}
		}
		
		
		motor_pid_cal(out);
		
		inn->info.target = out->info.out;	//Ŀ��ֵת�Ƶ��ٶȻ�
		
		/*--�ڻ�����--*/
		motor_pid_err(inn , meas2);  
		motor_pid_cal(inn);	
		
		return inn->info.out;	
	}
}

/**
 *	@brief	˫��pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_double(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, float tar, char err_cal_mode)
{

	out->info.target = tar;

	return motor_pid_ctrl(out,inn,meas1,meas2,err_cal_mode);

}


/**
 *	@brief	��pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_single(motor_pid_t *out, float meas1, float tar)
{

	out->info.target = tar;

	return motor_pid_ctrl(out,NULL,meas1,NULL,0);

}


/**
 *	@brief	λ��pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_position(struct motor_class_t *motor,float target)//
{
	
	if(motor->state.init_flag == M_DEINIT)
	{
		return 0;
	}
	
	if(motor->pid.position.info.init_flag == M_DEINIT || motor->pid.position_in.info.init_flag == M_DEINIT)
	{
		return 0;
	}	
	
	motor->pid.position.info.target = target;
	
	return motor_pid_ctrl(&motor->pid.position,&motor->pid.position_in,motor->rx_info.angle_sum,motor->rx_info.speed,0);
	
}


/**
 *	@brief	�Ƕ�pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_angle(struct motor_class_t *motor,float target)//8192
{
	
	if(motor->state.init_flag == M_DEINIT)
	{
		return 0;
	}

	if(motor->pid.angle.info.init_flag == M_DEINIT || motor->pid.angle_in.info.init_flag == M_DEINIT)
	{
		return 0;
	}	
	
	motor->pid.angle.info.target = target/22.75f;
	
	return motor_pid_ctrl(&motor->pid.angle,&motor->pid.angle_in,motor->rx_info.angle/22.75f,motor->rx_info.speed,1);
	
}



/**
 *	@brief	����pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_step(struct motor_class_t *motor,float target,char step)//8192
{
	
	if(motor->state.init_flag == M_DEINIT)
	{
		return 0;
	}

	if(motor->pid.step.info.init_flag == M_DEINIT || motor->pid.step_in.info.init_flag == M_DEINIT)
	{
		return 0;
	}	
	
	motor->pid.angle.info.target = target/22.75f;
	
	return motor_pid_ctrl(&motor->pid.step,&motor->pid.step_in,motor->rx_info.angle/22.75f,motor->rx_info.speed,step);
	
}


/**
 *	@brief	�ٶ�pid���� 
 *  @return ���ؼ�����
 */
float motor_pid_speed(struct motor_class_t *motor,float target)//rpm
{
	
	if(motor->state.init_flag == M_DEINIT)
	{
		return 0;
	}	

	if(motor->pid.speed.info.init_flag == M_DEINIT)
	{
		return 0;
	}	
	
	motor->pid.speed.info.target = target;
	
	return motor_pid_ctrl(&motor->pid.speed,NULL,motor->rx_info.speed,NULL,0);
	
}




/*-----------------------------------------------------------------
*�󽮵������
-----------------------------------------------------------------*/

/**
 *	@brief	��CAN�����ж�ȡ�����λ�÷���
 */
static uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = ((uint16_t)rxData[0] << 8| rxData[1]);
	return angle;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ת��ת�ٷ���
 */
static int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = ((uint16_t)rxData[2] << 8| rxData[3]);
	return speed;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ʵ��ת�ص�������
 */
static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = ((int16_t)rxData[4] << 8 | rxData[5]);
	return current;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ʵ�����ת��
 */
static int16_t CAN_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((int16_t)rxData[4] << 8 | rxData[5]);
	return torque;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ʵ���¶�
 */
static uint8_t CAN_GetMotorTemperature(uint8_t *rxData)
{
	uint8_t temperature;
	temperature = rxData[6];
	return temperature;
}

/**
 *	@brief	�󽮵��can������Ϣ
 *  @return 
 */
void get_rm_can_drvie(struct motor_class_t *motor)
{
	if(motor == NULL)
	{
		return;
	}
	
	motor->rx = &rm_motor_rx;
	motor->tx = &rm_motor_tx;			
	
	if(motor->id.motor_type == GM6020)
	{
			if((motor->id.rx_id - 0x205U) < 4)
				motor->id.tx_id = 0x1FF;
			else
				motor->id.tx_id = 0x2FF;
			
			motor->id.buff_p = (motor->id.rx_id - 0x205U)%4;
	}
	else	if(motor->id.motor_type == GM3510)
	{
			if((motor->id.rx_id - 0x205U) < 4)
				motor->id.tx_id = 0x1FF;
			else
				motor->id.tx_id = 0x2FF;
			
			motor->id.buff_p = (motor->id.rx_id - 0x205U)%4;
	}
	else if(motor->id.motor_type == RM3508)
	{
			if((motor->id.rx_id - 0x201U) < 4)
				motor->id.tx_id = 0x200;
			else
				motor->id.tx_id = 0x1FF;
			
			motor->id.buff_p = (motor->id.rx_id - 0x201U)%4;
	}		
	else if(motor->id.motor_type == RM3510)
	{
			if((motor->id.rx_id - 0x201U) < 4)
				motor->id.tx_id = 0x200;
			else
				motor->id.tx_id = 0x1FF;
			
			motor->id.buff_p = (motor->id.rx_id - 0x201U)%4;
	}		
	else if(motor->id.motor_type == RM2006)
	{
			if((motor->id.rx_id - 0x201U) < 4)
				motor->id.tx_id = 0x200;
			else
				motor->id.tx_id = 0x1FF;
			
			motor->id.buff_p = (motor->id.rx_id - 0x201U)%4;
	}			

}

/**
 *	@brief	can������Ϣ����
 *  @return
 */
void get_rm_info(struct motor_class_t *motor, uint8_t *rxBuf)
{
	int16_t err;

	motor_rx_info_t *motor_info = &motor->rx_info;
	
	motor_info->angle       = CAN_GetMotorAngle(rxBuf);	
	motor_info->speed       = CAN_GetMotorSpeed(rxBuf);
	motor_info->current     = CAN_GetMotorCurrent(rxBuf);
	motor_info->torque      = CAN_GetMotorTorque(rxBuf);	
	motor_info->temperature = CAN_GetMotorTemperature(rxBuf);

	if(!motor_info->angle_prev && !motor_info->angle_sum)
		err = 0;
	else
		err = motor_info->angle - motor_info->angle_prev;
	
	/* ����� */
	if(m_abs(err) > 4095)
	{
		/* 0�� -> 8191 */
		if(err >= 0)
			err += -8191;
		/* 8191�� -> 0 */
		else
			err += 8191;
	}
	motor_info->angle_sum += err;	
	
	motor_info->angle_prev = motor_info->angle;		
	
	if(motor->id.motor_type == GM3510)
	{
		//Ĭ��һ����
		motor_info->speed       = err/(8192*0.001f); // v = err/0.001f  T = 8192/v  rpm = 1/T  err/(8192*0.001f)
		motor_info->current     = CAN_GetMotorSpeed(rxBuf);
		motor_info->torque      = CAN_GetMotorSpeed(rxBuf);
	}
	
	
	motor->state.offline_cnt = 0;
}

