#ifndef _PID_H
#define _PID_H

#include "stm32f4xx_HAL.h"

typedef struct
{
	float ref;//输入：系统待调节量的给定值
	float fdb;//输入：系统待调节量的反馈值
	float inte;//积分值
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;

	float componentKiMax;

	float output;
	float outputMax;

}PID_Regulator_t;


extern PID_Regulator_t cloud_pitch_speed_pid;
extern PID_Regulator_t cloud_pitch_position_pid;
extern PID_Regulator_t cloud_yaw_speed_pid;
extern PID_Regulator_t cloud_yaw_position_pid;

void PID_Calc(PID_Regulator_t *pid);
void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKiMax,float outputMax);
void Cloud_Speed(void);
void Cloud_Position(void);




#endif
