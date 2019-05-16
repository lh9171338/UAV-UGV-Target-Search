#include "stm32f4xx.h" 


#define USART2_RX_BUF_LENGTH 128
#define USART2_TX_BUF_LENGTH 128
#define USART2_RX_PACKAGE_LENGTH 8
#define USART2_TX_PACKAGE_LENGTH 42


extern uint8_t usart2_rx_buf[USART2_RX_BUF_LENGTH];
extern uint8_t usart2_tx_buf[USART2_TX_BUF_LENGTH];


void usart2_config(float bound);



