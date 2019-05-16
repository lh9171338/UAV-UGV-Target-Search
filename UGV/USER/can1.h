#include "stm32f4xx.h" 


#define CAN_3510_M1_ID          0x201
#define CAN_3510_M2_ID          0x202
#define CAN_3510_M3_ID          0x203
#define CAN_3510_M4_ID          0x204

#define FILTER_BUF 5


extern s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
extern float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;


void Can1_Configuration(void);
void Send_motorvalue_CAN1_4(s16 data1 , s16 data2 , s16 data3 , s16 data4);
void Send_motorvalue_CAN1_3(s16 data1 , s16 data2 , s16 data3 , s16 data4);

