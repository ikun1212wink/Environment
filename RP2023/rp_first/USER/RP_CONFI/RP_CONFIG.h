#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/*--------------------------------------
˵���飺

2023.4.29 HXZP 1324146673@qq.com

��������ȫ�ǳ�����ģ�����
�����ǲ��ֳ���һЩ���������㷨

���������ӣ�
https://gitee.com/HaveALitttleSao/RP2021_HAL_Template/tree/master/

--------------------------------------

ȫ�ֺ궨����ƽ���

������Ҫ��ͷ�ļ��������Ҫ��c�ļ���������������ɱ������

--------------------------------------

DSPʹ������ļ�·���ķ�������Ҫʲô���ʲô��Ҫȫ���ӽ�ȥ��Ҳ��Ҫ��ѡkeil��DSP����

ʹ��cube����Ҫ��user�е�cmsis�ļ��滻drives�е�cmsis

--------------------------------------

#define MASTER 1U

����ѡ��ʹ�ܲ�ͬ�����أ������л���ƥ��������Լ�����

--------------------------------------

����������ϵ��
powerΪ���ص�Դλ�ã�����ͼ�ڷ�������ϵ
С���أ�
�������z�ᣬ����x�ᣬ����y��
 ------------
|            | 
|power       |
|            |
 ------------

�����أ�
�������z�ᣬ����x�ᣬ����y��
 ------------
|            | 
|            |
|power       |
|            |
|            |
 ------------

 ���������ƣ�x,y,z����תangle���Ƕ��ƣ�
 	#define IMU_POSE_ANGLE 0 
	
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    0
	
�������л�ͨ�ŷ�ʽ������һ�ε�
���ؿ����������imu��ʼ�����ɹ�
���ִ������޷�ʵ��SPIͨ��

-----------------------------------------*/



/* 0������ 1������  */
/*MASTER SERIAL NUMBER*/

#define MASTER 0U

/* imu װ����̬  */
#if MASTER == 0U
	#define IMU_POSE_ANGLE 0
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    0
	#define IMU_PID_KP     10.f
	
	#define IMU_PID_KP_CONTROL     0.1f
	
	#define IMU_POTOCAL_TYPE IMU_SPI
	
#endif

#if MASTER == 1U
	#define IMU_POSE_ANGLE 180
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    1
	#define IMU_POSE_AZ    0
	#define IMU_PID_KP     10.f	
	
	#define IMU_POTOCAL_TYPE IMU_IIC
#endif


/*Devices Frivers Enable*/
#define BMI_ENABLE    1U		
#define CAN_ENABLE    1U
#define USART_ENABLE  1U

/*Usart Select*/
#define VISION_USART  1U
#define JUDGE_USART   5U

/*Test Enable*/
#define USART_TEST    0U //Enable Usart Test

#define RM_MOTOR_TEST            1U //Enable Motor Test
#define RM_MOTOR_CAN_TYPR_TEST   1U //1 CAN1 2 CAN2
#define RM_MOTOR_CAN_ID_TEST     0x205U //id
#define RM_MOTOR_TYPE_TEST       1U //1 6020 2 3508 3 2006


#define RC_KEY_MONITOR       1U 




#endif




