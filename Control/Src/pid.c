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

void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte += pid->err[1];
		
	
	pid->componentKp  = pid->kp * pid->err[1];
	pid->componentKi  = pid->ki * pid->inte;
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
	
	if(pid->inte > 2000)//对积分项进行限制
		pid->inte = 2000;
	else if (pid->inte < -2000)
		pid->inte = -2000;
	
//	if(pid->componentKp > pid->componentKpMax)
//		pid->componentKp = pid->componentKpMax;
//	else if (pid->componentKp < -pid->componentKpMax)
//		pid->componentKp = -pid->componentKpMax;
	
	if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;

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

void Cloud_Speed(void)
{
	//PITCH
	cloud_pitch_speed_pid.fdb = sensor.gyro.radian.y;					//反馈参数：陀螺仪角速度
	cloud_pitch_speed_pid.ref = cloud_pitch_position_pid.output;			//设定参数：位置环输出
	PID_Calc(&cloud_pitch_speed_pid);
	
	//YAW
	cloud_yaw_speed_pid.fdb = sensor.gyro.radian.x;					//反馈参数：陀螺仪角速度
	cloud_yaw_speed_pid.ref = cloud_yaw_position_pid.output;			//设定参数：位置环输出
	PID_Calc(&cloud_yaw_speed_pid);
	
}

void Cloud_Position(void)
{
	//PITCH
	cloud_pitch_position_pid.fdb = cloud_para[0].Bmechanical_angle;					//反馈参数:云台电机角度
	cloud_pitch_position_pid.ref = pitch;										//设定参数
	PID_Calc(&cloud_pitch_position_pid);
	
	//YAW
	cloud_yaw_position_pid.fdb = cloud_para[1].Bmechanical_angle;					//反馈参数：云台电机角度
	cloud_yaw_position_pid.ref = yaw;										//设定参数：位置环输出
	PID_Calc(&cloud_yaw_position_pid);
	
}

void ALLPID_Init()
{
	PID_Init(&cloud_pitch_position_pid,	-7,	0,	0,	1000,	5000);
	PID_Init(&cloud_pitch_speed_pid,	2.3,0.003,0,	500,5000);
	PID_Init(&cloud_yaw_position_pid,	6,	0,	0,	1000,	5000);
	PID_Init(&cloud_yaw_speed_pid	,	-4.0,-0.006,0,	500,5000);
	
}
