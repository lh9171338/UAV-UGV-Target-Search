#include "usart2.h"
#include "can1.h"
#include "string.h"

typedef struct
{
  int16_t ax;
	int16_t ay;
	int16_t az;
	
	int16_t wx;
	int16_t wy;
	int16_t wz;
	
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	
	int16_t mx;
	int16_t my;
	int16_t mz;
	
}imu_data_t;

typedef struct
{
  int32_t lon;
	int32_t lat;
	
}gps_data_t;

typedef struct
{
  int16_t x;
	int16_t y;
	
  int16_t vx;
	int16_t vy;	
	
}odom_data_t;

void imu_task(void);
void ImuDataProcess(uint8_t *pData);
void send_imu_data(void);






