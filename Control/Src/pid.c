/**
  *@file timer.c
  *@date 2018-12-17
  *@author Vacuo.W
  *@brief 
  */
  
#include "pid.h"
#include "control.h"
#include "can.h"
#include "32mpu6050.h"

/************CLOUD***************/
PID_Regulator_t cloud_pitch_speed_pid;
PID_Regulator_t cloud_pitch_position_pid;
PID_Regulator_t cloud_yaw_speed_pid;
PID_Regulator_t cloud_yaw_position_pid;

PID_Regulator_t cloud_yaw_cheet_pid;
PID_Regulator_t cloud_pitch_cheet_pid;

void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte += pid->err[1];
		
	
	pid->componentKp  = pid->kp * pid->err[1];
	pid->componentKi  = pid->ki * pid->inte;
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
	
	if(pid->inte > pid->componentKiMax)//�Ի������������
		pid->inte = pid->componentKiMax;
	else if (pid->inte < -pid->componentKiMax)
		pid->inte = -pid->componentKiMax;
	
//	if(pid->componentKp > pid->componentKpMax)
//		pid->componentKp = pid->componentKpMax;
//	else if (pid->componentKp < -pid->componentKpMax)
//		pid->componentKp = -pid->componentKpMax;
//	
//	if(pid->componentKi > pid->componentKiMax)
//		pid->componentKi = pid->componentKiMax;
//	else if (pid->componentKi < -pid->componentKiMax)
//		pid->componentKi = -pid->componentKiMax;

//	if(pid->componentKd > pid->componentKdMax)
//		pid->componentKd = pid->componentKdMax;
//	else if (pid->componentKd < -pid->componentKdMax)
//		pid->componentKd = -pid->componentKdMax;
	
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}

void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKiMax,float outputMax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->inte = 0;

	pid->componentKiMax = componentKiMax;

	pid->outputMax = outputMax;
}

void Cloud_Speed(float pitch_ref, float yaw_ref)
{
	//PITCH
	cloud_pitch_speed_pid.fdb = sensor.gyro.radian.y;					//���������������ǽ��ٶ�
	cloud_pitch_speed_pid.ref = pitch_ref;			//�趨����
	PID_Calc(&cloud_pitch_speed_pid);
	
	//YAW
	cloud_yaw_speed_pid.fdb = sensor.gyro.radian.x;					//���������������ǽ��ٶ�
	cloud_yaw_speed_pid.ref = yaw_ref;			//�趨����
	PID_Calc(&cloud_yaw_speed_pid);
	
}

void Cloud_Position(float pitch_ref, float yaw_ref)
{
	//PITCH
	cloud_pitch_position_pid.fdb = cloud_para[0].Bmechanical_angle;				//��������:��̨����Ƕ�
	cloud_pitch_position_pid.ref = pitch_ref;										//�趨����
	PID_Calc(&cloud_pitch_position_pid);
	
	//YAW
	cloud_yaw_position_pid.fdb = cloud_para[1].Bmechanical_angle;					//������������̨����Ƕ�
	cloud_yaw_position_pid.ref = yaw_ref;										//�趨������λ�û����
	PID_Calc(&cloud_yaw_position_pid);
	
}

void Cloud_Cheet(float pitch_ref, float yaw_ref)
{
	//PITCH
	cloud_pitch_cheet_pid.fdb = pitch_ref;
	cloud_pitch_cheet_pid.ref = 0;
	PID_Calc(&cloud_pitch_cheet_pid);
	
	//YAW
	cloud_yaw_cheet_pid.fdb = yaw_ref;
	cloud_yaw_cheet_pid.ref = 0;
	PID_Calc(&cloud_yaw_cheet_pid);
}

void ALLPID_Init()
{
	PID_Init(&cloud_pitch_position_pid,	-70,	-1,	5,	3000,	10000);
	//PID_Init(&cloud_pitch_speed_pid,	2.3,0.003,0,	500,3000);
	PID_Init(&cloud_pitch_speed_pid,	0.30, 0, -0,	2000,5000);
	
	PID_Init(&cloud_yaw_position_pid,	45,	0,	-7,	1000,	12000);
	//PID_Init(&cloud_yaw_speed_pid	,	-4.0,-0.006,0,	500,3000);
	PID_Init(&cloud_yaw_speed_pid	,	-0.5,-0,-0,	1000,5000);
	
	PID_Init(&cloud_yaw_cheet_pid	,	45,0.1,-6,	1000,12000);
	PID_Init(&cloud_pitch_cheet_pid	,	50,0.4,-4,	3000,30000);
	
}
