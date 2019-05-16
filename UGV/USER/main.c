#include "sys.h"
#include "delay.h"
#include "can1.h"
#include "usart1.h" 
#include "usart2.h" 
#include "usart6.h" 
#include "pid.h"
#include "timer.h"
#include "led.h"


int main(void)
{ 
 	SystemInit();
	led_config();
	Can1_Configuration();
	delay_ms(1000);
	usart1_config(100000); // receive remote data
	usart2_config(115200); // receive control data and send imu data  
	usart6_config(9600); // receive imu data from mpu9250
  PID_InitALL();
	Tim2_config(); // chassis move timer
	Tim4_config(); // send IMU data timer
	
	while(1)
	{
	}
	return 0;
}
 

