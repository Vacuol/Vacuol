#ifndef _PID_H
#define _PID_H

#include "stm32f4xx_HAL.h"

typedef struct
{
	float ref;//���룺ϵͳ���������ĸ���ֵ
	float fdb;//���룺ϵͳ���������ķ���ֵ
	float inte;//����ֵ
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

extern PID_Regulator_t cloud_yaw_cheet_pid;
extern PID_Regulator_t cloud_pitch_cheet_pid;


void PID_Calc(PID_Regulator_t *pid);
void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKiMax,float outputMax);
void Cloud_Speed(float pitch_ref, float yaw_ref);
void Cloud_Position(float pitch_ref, float yaw_ref);
void Cloud_Cheet(float pitch_ref, float yaw_ref);
void ALLPID_Init(void);




#endif
