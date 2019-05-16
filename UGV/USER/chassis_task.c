#include "chassis_task.h"


int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;
s16 chassis_Motor_M1_set , chassis_Motor_M2_set , chassis_Motor_M3_set , chassis_Motor_M4_set;


/////////////////////////////////////////////////////////////////
void chassis_task(void)
{
	switch(RemoteCtrlData.s2)
	{
	  //stop 或 遥控未开启：		
		case STOP:
		case NULL:
			Stop_chassis_control();
		break;
	  //遥控模式
		case REMOTE_INPUT:
			remote_chassis_control_mode();
		  break;
	  //自动模式
		case AUTOMATIC_INPUT:
			Automatic_chassis_control();
		  break;
		default:
			Stop_chassis_control();
		break;
	}
}
 
/////////////////////////////////////////////////////////////////
void Stop_chassis_control()
{
	rc_ch0_value = 0;
	rc_ch1_value = 0;
	rc_ch2_value = 0;
	rc_ch3_value = 0;
	chassis_Motor_M1_set = 0; 
	chassis_Motor_M2_set = 0; 
	chassis_Motor_M3_set = 0; 
	chassis_Motor_M4_set = 0; 
	Send_motorvalue_CAN1_4(0,0,0,0);	
}
/////////////////////////////////////////////////////////////////
const float param_ch0_1 = 0.2 , param_ch2_3 = 0.21;
void remote_chassis_control_mode()
{
	rc_ch0_value = (RemoteCtrlData.ch0 - 1024) * param_ch0_1;
	rc_ch1_value = (RemoteCtrlData.ch1 - 1024) * param_ch0_1;
	rc_ch2_value = (RemoteCtrlData.ch2 - 1024) * param_ch2_3;
	rc_ch3_value = (RemoteCtrlData.ch3 - 1024) * param_ch2_3;
	
	//底盘运动解算
	chassis_Motor_M1_set = -rc_ch1_value + rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M2_set =  rc_ch1_value + rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M3_set =  rc_ch1_value - rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M4_set = -rc_ch1_value - rc_ch0_value + rc_ch2_value; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));  
}

///////////////////////////////////////////////////
const float param_vxy = 0.119 , param_vyaw = 0.048;
void Automatic_chassis_control()
{
	rc_ch1_value =  AutoCtrlData.velocity_x * param_vxy;
	rc_ch0_value = -AutoCtrlData.velocity_y * param_vxy;
	rc_ch2_value = -AutoCtrlData.velocity_yaw * param_vyaw;
				
	 /////////////////
	//底盘运动解算
	chassis_Motor_M1_set = -rc_ch1_value + rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M2_set =  rc_ch1_value + rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M3_set =  rc_ch1_value - rc_ch0_value + rc_ch2_value; 
	chassis_Motor_M4_set = -rc_ch1_value - rc_ch0_value + rc_ch2_value; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set)); 
}













