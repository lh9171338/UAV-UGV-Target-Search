#include "callback.h"

/////////////////////////////////////////////////////////////////
//Chassis move
void TIM2_IRQHandler(void) // 1000Hz
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		chassis_task();		
	}
}

/////////////////////////////////////////////////////////////////
//Send IMU data
void TIM4_IRQHandler(void) //10Hz
{
	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
		imu_task();
	}
}

/////////////////////////////////////////////////////////////////
//Receive remote data
void USART1_IRQHandler(void)
{
	uint32_t package_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART1->SR;
		(void)USART1->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			package_len = USART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)USART1_RX_BUF_LENGTH;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
      if(package_len == USART1_RX_PACKAGE_LENGTH)
			{
				RemoteCtrlDataProcess(usart1_rx_buf[0]);
			}
			DMA_Cmd(DMA2_Stream2, ENABLE);
		}
		//Target is Memory1
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			package_len = USART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)USART1_RX_BUF_LENGTH;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
      if(package_len == USART1_RX_PACKAGE_LENGTH)
			{
				RemoteCtrlDataProcess(usart1_rx_buf[1]);
			}
			DMA_Cmd(DMA2_Stream2, ENABLE);
		}
	}       
}

/////////////////////////////////////////////////////////////////
// Receive control data
void USART2_IRQHandler(void)
{
	uint32_t package_len = 0;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART2->SR;
		(void)USART2->DR;
		
    DMA_Cmd(DMA1_Stream5,DISABLE);  
    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);  
    package_len = USART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);
		if(package_len == USART2_RX_PACKAGE_LENGTH && usart2_rx_buf[0] == 0x7F 
			&& usart2_rx_buf[USART2_RX_PACKAGE_LENGTH - 1] == 0x7E)
		{
			AutoCtrlDataProcess(usart2_rx_buf);
		}
    DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_BUF_LENGTH);  
    DMA_Cmd(DMA1_Stream5, ENABLE);  
	}
}
			
/////////////////////////////////////////////////////////////////
// receive imu data from mpu9250
void USART6_IRQHandler(void)
{
	uint32_t package_len = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART6->SR;
		(void)USART6->DR;
		
    DMA_Cmd(DMA2_Stream1,DISABLE);  
    DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF5);  
    package_len = USART6_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1);
		if(package_len == USART6_RX_PACKAGE_LENGTH)
		{
			ImuDataProcess(usart6_rx_buf);
		}
    DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_BUF_LENGTH);  
    DMA_Cmd(DMA2_Stream1, ENABLE);  
	}
}

