#include "imu_task.h"

imu_data_t imu_data;
gps_data_t gps_data;
odom_data_t odom_data;

/////////////////////////////////////////////////////////////////
void imu_task(void)
{
	
	float x = 0.0001 * (-chassis_Motor1_loco + chassis_Motor3_loco + chassis_Motor2_loco - chassis_Motor4_loco) / 4.0 * 1.41421 / 2.0 * 0.822; 
	float y = 0.0001 * (-chassis_Motor1_loco + chassis_Motor3_loco - chassis_Motor2_loco + chassis_Motor4_loco) / 4.0 * 1.41421 / 2.0 * 0.822;
	float vx = 0.01 * (-chassis_Motor_M1[1] + chassis_Motor_M3[1] + chassis_Motor_M2[1] - chassis_Motor_M4[1]) / 4.0 * 1.41421 / 2.0 * 1.2; 
	float vy = 0.01 * (-chassis_Motor_M1[1] + chassis_Motor_M3[1] - chassis_Motor_M2[1] + chassis_Motor_M4[1]) / 4.0 * 1.41421 / 2.0 * 1.2;
	odom_data.x = (int16_t)(x * 100);
	odom_data.y = (int16_t)(y * 100);
	odom_data.vx = (int16_t)(vx * 10000);
	odom_data.vy = (int16_t)(vy * 10000);

	send_imu_data();
}

/////////////////////////////////////////////////////////////////
void ImuDataProcess(uint8_t *pData)
{
	memcpy(&imu_data.ax, pData + 11 * 0 + 2, 6);
	memcpy(&imu_data.wx, pData + 11 * 1 + 2, 6);
	memcpy(&imu_data.roll, pData + 11 * 2 + 2, 6);
	memcpy(&imu_data.mx, pData + 11 * 3 + 2, 6);
	memcpy(&gps_data.lon, pData + 11 * 4 + 2, 8);

}

/////////////////////////////////////////////////////////////////
void send_imu_data(void)
{
	usart2_tx_buf[0]  = 0x7F; // head of the package

	//imu data
	memcpy(usart2_tx_buf + 1, &imu_data.ax, 24);

	//GPS data
	memcpy(usart2_tx_buf + 25, &gps_data.lon, 8);
	
  //wheel odonmetry data
	memcpy(usart2_tx_buf + 33, &odom_data.x, 8);
	
	usart2_tx_buf[41] = 0x7E; // tail of the package
	for(int i = 0 ; i < USART2_TX_PACKAGE_LENGTH ; i++)
	{
		USART_SendData(USART2 , usart2_tx_buf[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC ) != SET);
	}
}
