#include "remote_task.h"
 
RC_t RemoteCtrlData = {0};
AC_t AutoCtrlData = {0};

/////////////////////////////////////////////////////////////////
void RemoteCtrlDataProcess(uint8_t *pData)
{
	if(pData == NULL)
	{
		return;
	}
	
	RemoteCtrlData.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; // left or right(11 bits)
	RemoteCtrlData.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; // forward or backward
	RemoteCtrlData.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF; // turn left or rigth 
	RemoteCtrlData.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF; // up or down

	RemoteCtrlData.s1  = ((pData[5] >> 4) & 0x000C) >> 2; // left switch
	RemoteCtrlData.s2  = ((pData[5] >> 4) & 0x0003); // right switch
}
 
/////////////////////////////////////////////////////////////////
void AutoCtrlDataProcess(uint8_t *pData)
{
	int16_t vx = (int16_t)(pData[1] | (pData[2] << 8));
	int16_t vy = (int16_t)(pData[3] | (pData[4] << 8));
	int16_t vyaw = (int16_t)(pData[5] | (pData[6] << 8));
	AutoCtrlData.velocity_x   = (int16_t)((float)vx / 10000 * 1024); 
	AutoCtrlData.velocity_y   = (int16_t)((float)vy / 10000 * 1024); 
	AutoCtrlData.velocity_yaw = (int16_t)((float)vyaw / 10000 * 1024); 
}
