#include "usart6.h" 

uint8_t usart6_rx_buf[USART6_RX_BUF_LENGTH];
uint8_t usart6_tx_buf[USART6_TX_BUF_LENGTH];

void usart6_config(float bound)
{
  GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef  dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOA时钟
	
	GPIO_PinAFConfig(GPIOG , GPIO_PinSource9  , GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG , GPIO_PinSource14 , GPIO_AF_USART6);
		
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG , &gpio);
	
  USART_DeInit(USART6);
  USART_StructInit(&usart);
	usart.USART_BaudRate = bound;//波特率设置
	usart.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	usart.USART_StopBits = USART_StopBits_1;//一个停止位
	usart.USART_Parity = USART_Parity_No;//无奇偶校验位
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;	//收发模式
  USART_Init(USART6, &usart); 
	
	USART_DMACmd(USART6 , USART_DMAReq_Rx , ENABLE);

	DMA_DeInit(DMA2_Stream1);  
	dma.DMA_Channel = DMA_Channel_5;   
	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	dma.DMA_Memory0BaseAddr = (uint32_t)usart6_rx_buf;  
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	dma.DMA_BufferSize = USART6_RX_BUF_LENGTH;  
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;   
	dma.DMA_Mode = DMA_Mode_Circular;  
	dma.DMA_Priority = DMA_Priority_Low;  
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;           
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
			 
	DMA_Init(DMA2_Stream1, &dma);    
	DMA_Cmd(DMA2_Stream1,ENABLE);  

	nvic.NVIC_IRQChannel = USART6_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 1;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART6, ENABLE);
}




