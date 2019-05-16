#include "stm32f4xx.h" 


#define NULL 0
#define REMOTE_INPUT 1
#define AUTOMATIC_INPUT 2
#define STOP 3

typedef struct
{
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;	
	
 uint16_t s1;
 uint16_t s2;	
}RC_t; // Remote Control struct

typedef struct
{
	int16_t velocity_x;
	int16_t velocity_y;
	int16_t velocity_yaw;
}AC_t; // Automatic Control struct

extern RC_t RemoteCtrlData;
extern AC_t AutoCtrlData;

void RemoteCtrlDataProcess(uint8_t *pData);
void AutoCtrlDataProcess(uint8_t *pData);



