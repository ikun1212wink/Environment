/**
 * @file        vision_sensor.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        18-February-2021
 * @brief       About the Pathway.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "RP_CONFIG.h"

#include "vision.h"
#include "string.h"
#include "cmsis_os.h"
#include "uart_drv.h"

static void vision_update(vision_t *vision, uint8_t *rxBuf);
static void vision_init(vision_t *vision);
static void vision_heart_beat(vision_t *vision);
static void vision_sendBuff(struct vision_struct *self,uint8_t *Data);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


// �Ӿ�������
vision_t	vision = {
	
	.info.offline_max_cnt = 200,
	.info.offline_cnt     = 0,
	.info.state = VISION_OFFLINE,
	
	.init       = vision_init,
	.update     = vision_update,
	.heart_beat = vision_heart_beat,
  .send       = vision_sendBuff,
	
};

/* Private functions ---------------------------------------------------------*/


static void vision_init(vision_t *vision)
{
	// ��ʼ��Ϊ����״̬
	vision->info.offline_cnt = vision->info.offline_max_cnt + 1;
	vision->info.state = VISION_OFFLINE;

}



static void vision_heart_beat(vision_t *vision_sen)
{
	vision_info_t *vision_info = &vision_sen->info;

	vision_info->offline_cnt++;
	if(vision_info->offline_cnt > vision_info->offline_max_cnt) 
	{
		vision_info->offline_cnt = vision_info->offline_max_cnt;
		vision_info->state = VISION_OFFLINE;
	} 
	else 
	{
		/* ����->���� */
		if(vision_info->state == VISION_OFFLINE)
			vision_info->state = VISION_ONLINE;
	}	
	
}

static void vision_sendBuff(struct vision_struct *self,uint8_t *Data)
{
	UART_Send(Data,VISION_USART);
}


/**
 *	@brief	�Ӿ����ݽ���Э��
 */
void vision_update(vision_t *vision_sen, uint8_t *rxBuf)
{
	vision_data_t *vision_data = &vision_sen->data;
	
	uint8_t res = false;
	vision_data->State.rx_cnt++;
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[sof] == VISION_FRAME_HEADER) 
	{	
		res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
		/* ֡ͷCRC8У��*/
		if(res == true)
		{
			res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
			/* ֡βCRC16У�� */
			if(res == true) 
			{
				/* ������ȷ�򿽱����հ� */
				memcpy(vision_data->RxPacket.RxData.data, rxBuf, LEN_VISION_RX_PACKET);
				vision_data->State.rx_data_update = true;	// �Ӿ����ݸ���	

				memcpy((void*)(vision_data->RxPacket.RxData.jiesuan),vision_data->RxPacket.RxData.data+3,LEN_VISION_RX_PACKET-5);

				/* ֡�ʼ��� */
				vision_data->State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_data->State.rx_time_fps  = vision_data->State.rx_time_now - vision_data->State.rx_time_prev;
				vision_data->State.rx_time_prev = vision_data->State.rx_time_now;		
				vision_data->State.offline_cnt  = 0;
			}
		}
	}	
	/* ������Ч���ж� */
	if(res == true) 
	{
		vision_data->State.rx_data_valid = true;
	} 
	else if(res == false) 
	{
		vision_data->State.rx_data_valid = false;
		vision_data->State.rx_err_cnt++;
	}

	memcpy(&vision_sen->data.RxPacket,vision_sen->data.RxPacket.RxData.data, LEN_VISION_RX_PACKET);

	
}
/* Exported functions --------------------------------------------------------*/
