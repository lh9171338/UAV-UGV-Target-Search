#include "timer.h"
 
void Tim2_config(void)
{
    TIM_TimeBaseInitTypeDef   tim;
    NVIC_InitTypeDef          nvic; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //PCLK1=42MHz,TIM2 clk =84MHz
 
    /* TIM2 */
    tim.TIM_Prescaler = 84 - 1; // 84MHz / 84 = 1MHz
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000; // 1000 / 1MHz = 1ms   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&tim);
		
	  nvic.NVIC_IRQChannel = TIM2_IRQn;	  
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 0;	
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		 
	  TIM_ARRPreloadConfig(TIM2,ENABLE); 
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIM2,ENABLE);
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
}

void Tim4_config(void)
{
    TIM_TimeBaseInitTypeDef   tim;
    NVIC_InitTypeDef          nvic; 

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   //PCLK1=42MHz,TIM4 clk =84MHz
		
    /* TIM4 */
		tim.TIM_Prescaler = 8400 - 1; // 84MHz / 8400 = 10kHz
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000; // 1000 / 10kHz = 100ms   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&tim);
		
	  nvic.NVIC_IRQChannel = TIM4_IRQn;	  
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;	
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
         
    TIM_ARRPreloadConfig(TIM4,ENABLE);
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIM4,ENABLE);
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
}

