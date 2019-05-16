#include "usart2.h" 

uint8_t usart2_rx_buf[USART2_RX_BUF_LENGTH];
uint8_t usart2_tx_buf[USART2_TX_BUF_LENGTH];

void usart2_config(float bound)
{
  GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart2;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef  dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); //使能GPIOA时钟
	
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource5 , GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource6 , GPIO_AF_USART2);
		
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD , &gpio);
	
  USART_DeInit(USART2);
  USART_StructInit(&usart2);
	usart2.USART_BaudRate = bound;//波特率设置
	usart2.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	usart2.USART_StopBits = USART_StopBits_1;//一个停止位
	usart2.USART_Parity = USART_Parity_No;//无奇偶校验位
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  usart2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;	//收发模式
  USART_Init(USART2, &usart2); //初始化串口1
	
	USART_DMACmd(USART2 , USART_DMAReq_Rx , ENABLE);

	DMA_DeInit(DMA1_Stream5);  
	dma.DMA_Channel = DMA_Channel_4;   
	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	dma.DMA_Memory0BaseAddr = (uint32_t)usart2_rx_buf;  
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	dma.DMA_BufferSize = USART2_RX_BUF_LENGTH;  
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;   
	dma.DMA_Mode = DMA_Mode_Circular;  
	dma.DMA_Priority = DMA_Priority_VeryHigh;  
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;           
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
			 
	DMA_Init(DMA1_Stream5, &dma);    
	DMA_Cmd(DMA1_Stream5,ENABLE);  

	nvic.NVIC_IRQChannel = USART2_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 1;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART2, ENABLE);
}
 







