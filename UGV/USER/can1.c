#include "can1.h"


void Can1_Configuration(void)
{
	CAN_InitTypeDef can1;
	GPIO_InitTypeDef       gpio;
	CAN_FilterInitTypeDef  can_filter;
	NVIC_InitTypeDef       nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &gpio);
	
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource0 , GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource1 , GPIO_AF_CAN1);
	
	can1.CAN_TTCM = DISABLE;
	can1.CAN_ABOM = DISABLE;
	can1.CAN_AWUM = DISABLE;
	can1.CAN_NART = ENABLE;
	can1.CAN_RFLM = DISABLE;
	can1.CAN_TXFP = ENABLE;
	can1.CAN_Mode = CAN_Mode_Normal;
	can1.CAN_SJW  = CAN_SJW_1tq;
	can1.CAN_BS1 = CAN_BS1_9tq;
	can1.CAN_BS2 = CAN_BS2_4tq;
	can1.CAN_Prescaler = 3;        
	CAN_Init(CAN1, &can1); //  180M / 4 = 45M , 45M / 3 = 15M , 15M / 
 
	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;	
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;	
	can_filter.CAN_FilterFIFOAssignment=0;		
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);

	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	
	CAN_ITConfig(CAN1,CAN_IT_TME ,ENABLE);		
}

/////////////////////////////////////////////////////////////////
void Send_motorvalue_CAN1_4(s16 data1 , s16 data2 , s16 data3 , s16 data4)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x200;							//报文ID号
	tx_message.RTR = CAN_RTR_Data;				//数据帧
	tx_message.IDE = CAN_Id_Standard;			//标准ID
	tx_message.DLC = 0x08;								//数据字节数

  tx_message.Data[0] = data1 >> 8;
  tx_message.Data[1] = data1  ;
  tx_message.Data[2] = data2 >> 8;
  tx_message.Data[3] = data2  ;
  tx_message.Data[4] = data3 >> 8;
  tx_message.Data[5] = data3  ;
  tx_message.Data[6] = data4 >> 8;
  tx_message.Data[7] = data4  ;
	
	CAN_Transmit(CAN1,&tx_message);
}

/////////////////////////////////////////////////////////////////
void Send_motorvalue_CAN1_3(s16 data1 , s16 data2 , s16 data3 , s16 data4)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x1ff	;					//报文ID号
	tx_message.RTR = CAN_RTR_Data;				//数据帧
	tx_message.IDE = CAN_Id_Standard;			//标准ID
	tx_message.DLC = 0x08;								//数据字节数

  tx_message.Data[0] = data1 >> 8;
  tx_message.Data[1] = data1  ;
  tx_message.Data[2] = data2 >> 8;
  tx_message.Data[3] = data2  ;
  tx_message.Data[4] = data3 >> 8;
  tx_message.Data[5] = data3  ;
  tx_message.Data[6] = data4 >> 8;
  tx_message.Data[7] = data4  ;
	
	CAN_Transmit(CAN1,&tx_message);
}

	
/////////////////////////////////////////////////////////////////
// Receive speed of the motor
int startcan1_flag_cnt = 0;
int chassismotor_cnt1 , chassismotor_cnt2 , chassismotor_cnt3 , chassismotor_cnt4;
//底盘电机参数：
s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
s16 chassis_Motor_M1ofset, chassis_Motor_M2ofset, chassis_Motor_M3ofset, chassis_Motor_M4ofset;
s16 chassis_Motor_M1_last, chassis_Motor_M2_last, chassis_Motor_M3_last, chassis_Motor_M4_last;
s16 chassis_Motor_M1_cnt , chassis_Motor_M2_cnt , chassis_Motor_M3_cnt , chassis_Motor_M4_cnt;
int32_t chassis_Motor_M1_buf[5] , chassis_Motor_M2_buf[5] , chassis_Motor_M3_buf[5] , chassis_Motor_M4_buf[5];
float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);		
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);	
		
		if(startcan1_flag_cnt < 1000)
		{
			startcan1_flag_cnt++;
		}
		if(startcan1_flag_cnt >= 1000)
		{
			startcan1_flag_cnt = 1000;
		}
			
		switch(rx_message.StdId)
		{
			case CAN_3510_M1_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M1[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M1ofset = chassis_Motor_M1[0];
				}
				else
				{
					chassis_Motor_M1_last = chassis_Motor_M1[0];
					chassis_Motor_M1[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M1[0] - chassis_Motor_M1_last > 4096)
					{
						chassis_Motor_M1_cnt--;
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last - 8192;
					}
					else if(chassis_Motor_M1[0] - chassis_Motor_M1_last < -4096)
					{
						chassis_Motor_M1_cnt++;
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last + 8192;
					}
					else 
					{
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last ;
					}
					chassis_Motor1_loco = chassis_Motor_M1_cnt * 8192 + chassis_Motor_M1[0] - chassis_Motor_M1ofset;					
				}
			  int32_t temp_sum_1 = 0;
				chassis_Motor_M1_buf[chassismotor_cnt1++] = chassis_Motor_M1[3];
        if(chassismotor_cnt1 >= FILTER_BUF) 
				{
					chassismotor_cnt1 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_1 += chassis_Motor_M1_buf[i];
				}
				chassis_Motor_M1[1] = (int16_t)(temp_sum_1 / FILTER_BUF * 7.324f);
			  break;
			case CAN_3510_M2_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M2[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M2ofset = chassis_Motor_M2[0];
				}
				else
				{
					chassis_Motor_M2_last = chassis_Motor_M2[0];
					chassis_Motor_M2[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M2[0] - chassis_Motor_M2_last > 4096)
					{
						chassis_Motor_M2_cnt--;
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last - 8192;
					}
					else if(chassis_Motor_M2[0] - chassis_Motor_M2_last < -4096)
					{
						chassis_Motor_M2_cnt++;
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last + 8192;
					}
					else 
					{
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last ;
					}
					chassis_Motor2_loco = chassis_Motor_M2_cnt * 8192 + chassis_Motor_M2[0] - chassis_Motor_M2ofset;					
				}
			  int32_t temp_sum_2 = 0;
				chassis_Motor_M2_buf[chassismotor_cnt2++] = chassis_Motor_M2[3];
        if(chassismotor_cnt2 >= FILTER_BUF) 
				{
					chassismotor_cnt2 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_2 += chassis_Motor_M2_buf[i];
				}
				chassis_Motor_M2[1] = (int16_t)(temp_sum_2 / FILTER_BUF * 7.324f);
			  break;	
			case CAN_3510_M3_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M3[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M3ofset = chassis_Motor_M3[0];
				}
				else
				{
					chassis_Motor_M3_last = chassis_Motor_M3[0];
					chassis_Motor_M3[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M3[0] - chassis_Motor_M3_last > 4096)
					{
						chassis_Motor_M3_cnt--;
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last - 8192;
					}
					else if(chassis_Motor_M3[0] - chassis_Motor_M3_last < -4096)
					{
						chassis_Motor_M3_cnt++;
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last + 8192;
					}
					else 
					{
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last ;
					}
					chassis_Motor3_loco = chassis_Motor_M3_cnt * 8192 + chassis_Motor_M3[0] - chassis_Motor_M3ofset;					
				}
			  int32_t temp_sum_3 = 0;
				chassis_Motor_M3_buf[chassismotor_cnt3++] = chassis_Motor_M3[3];
        if(chassismotor_cnt3 >= FILTER_BUF) 
				{
					chassismotor_cnt3 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_3 += chassis_Motor_M3_buf[i];
				}
				chassis_Motor_M3[1] = (int16_t)(temp_sum_3 / FILTER_BUF * 7.324f);
			  break;					
			case CAN_3510_M4_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M4[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M4ofset = chassis_Motor_M4[0];
				}
				else
				{
					chassis_Motor_M4_last = chassis_Motor_M4[0];
					chassis_Motor_M4[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M4[0] - chassis_Motor_M4_last > 4096)
					{
						chassis_Motor_M4_cnt--;
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last - 8192;
					}
					else if(chassis_Motor_M4[0] - chassis_Motor_M4_last < -4096)
					{
						chassis_Motor_M4_cnt++;
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last + 8192;
					}
					else 
					{
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last ;
					}
					chassis_Motor4_loco = chassis_Motor_M4_cnt * 8192 + chassis_Motor_M4[0] - chassis_Motor_M4ofset;					
				}
			  int32_t temp_sum_4 = 0;
				chassis_Motor_M4_buf[chassismotor_cnt4++] = chassis_Motor_M4[3];
        if(chassismotor_cnt4 >= FILTER_BUF) 
				{
					chassismotor_cnt4 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_4 += chassis_Motor_M4_buf[i];
				}
				chassis_Motor_M4[1] = (int16_t)(temp_sum_4 / FILTER_BUF * 7.324f);
			  break;			
			}				
	}
}
	
	
	
	
	
	
	


