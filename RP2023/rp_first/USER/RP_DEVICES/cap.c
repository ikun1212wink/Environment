#include "cap.h"
/*
����ʹ�ã�
1��ʹ��cap.setdata�������и�ֵ
2����can��Buff0x2E��Buff0x2F���������ͳ�ȥ
����һ�������ݰ�һ���ǿ��ư������Ƶ�Ƶ���Լ�����
*/


static void CAP_setMessage(cap_t *self,float powerBuff,float powerLimit,float volt,float current);
static void CAP_rxMessage(cap_t *self,uint32_t canId, uint8_t *rxBuf);
static void CAP_heart_beat(cap_t *self);
static void CAP_ModifyOutInLimit(cap_t *self,float out,float in);
cap_t cap = {

	.info.canId = 0x30,
	
	.info.offline_max_cnt = 100,
	.info.offline_cnt = 0,
	.info.state = CAP_OFFLINE,

	.data.tx.output_power_limit = 150,
	.data.tx.input_power_limit  = 150,

	.modifyLimit = CAP_ModifyOutInLimit,
	.updata      = CAP_rxMessage,
	.setdata     = CAP_setMessage,
	.heart_beat  = CAP_heart_beat,
};

static void CAP_heart_beat(cap_t *self)
{
	cap_info_t *info = &self->info;

	info->offline_cnt++;
	if(info->offline_cnt > info->offline_max_cnt)//ÿ�εȴ�һ��ʱ����Զ�����
	{
		info->offline_cnt = info->offline_max_cnt;
		info->state = CAP_OFFLINE;
	} 
	else //ÿ�ν��ճɹ�����ռ���
	{
		/* ����->���� */
		if(info->state == CAP_OFFLINE)
		{
			info->state = CAP_ONLINE;
		}
	}
}	


int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //��0.5ʹ����ȡ�������������
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}

//����ϵͳ���͹��ʻ����Ƶ��Ϊ50Hz���ú�����Ҫÿ20ms����һ�Ρ�
void CAP_setBuff0x2E(cap_t *self)
{
    self->data.Buff0x2E[0] = self->data.tx.chassis_power_buffer;    //���̹��ʻ��壬0~60J
    self->data.Buff0x2E[1] = self->data.tx.chassis_volt;    	//���������ѹ ��λ ���� **
    self->data.Buff0x2E[2] = self->data.tx.chassis_current;    //����������� ��λ ���� **
}

void CAP_setBuff0x2F(cap_t *self)
{
    self->data.Buff0x2F[0] = self->data.tx.chassis_power_limit;   //���̹����������ޣ�0~120W
    self->data.Buff0x2F[1] = self->data.tx.output_power_limit;    //���ݷŵ繦�����ƣ�-120~300W
    self->data.Buff0x2F[2] = self->data.tx.input_power_limit;     //���ݳ�繦�����ƣ�0~150W
    self->data.Buff0x2F[3] = self->data.tx.cap_control.all;       //���ݿ��أ�0���رգ���1��������

}

void CAP_ModifyOutInLimit(cap_t *self,float out,float in)
{
	self->data.tx.input_power_limit = in;
	self->data.tx.input_power_limit = out;
}

void CAP_setMessage(cap_t *self,float powerBuff,float powerLimit,float volt,float current)
{
    self->data.tx.chassis_power_buffer = powerBuff;
    self->data.tx.chassis_power_limit  = powerLimit;
    self->data.tx.chassis_volt         = volt;
    self->data.tx.chassis_current      = current;
	
    self->data.tx.cap_control.bit.cap_switch = 1;
    self->data.tx.cap_control.bit.cap_record = 0;
	
		CAP_setBuff0x2E(self);
		CAP_setBuff0x2F(self);
}



void CAP_rxMessage(cap_t *self,uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x30)
    {
        self->data.rx.cap_Ucr = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
        self->data.rx.cap_I   = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
        self->data.rx.cap_state.state = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);    
			
        self->info.offline_cnt = 0;			
    }
}









