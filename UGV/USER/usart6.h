#include "stm32f4xx.h" 


#define USART6_RX_BUF_LENGTH 128
#define USART6_TX_BUF_LENGTH 128
#define USART6_RX_PACKAGE_LENGTH 55
#define USART6_TX_PACKAGE_LENGTH 0


extern uint8_t usart6_rx_buf[USART6_RX_BUF_LENGTH];
extern uint8_t usart6_tx_buf[USART6_TX_BUF_LENGTH];


void usart6_config(float bound);



