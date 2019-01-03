/**
  *@file timer.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "timer.h"
#include "control.h"
#include "can.h"
#include "pid.h"
#include "32mpu6050.h"
uint16_t Timetick1ms = 0;

void Test_task(void)
{
	Timetick1ms++;
	get_mpu_data();

	if(tele_data.s1 == 2)
	{
		Cloud_Speed(cloud_pitch_position_pid.output, cloud_yaw_position_pid.output);
	
		cloud_para[0].motor_output=cloud_pitch_speed_pid.output;
		cloud_para[1].motor_output=cloud_yaw_speed_pid.output;
		cloud_motor_output(cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);
		
		if (Timetick1ms%10==0)
		{
			Cloud_Position(pitch, yaw);
		}
	}
	else if(tele_data.s1 == 3)
	{
		Cloud_Speed(cloud_pitch_cheet_pid.output, cloud_yaw_cheet_pid.output);
	
		cloud_para[0].motor_output=cloud_pitch_speed_pid.output;
		cloud_para[1].motor_output=cloud_yaw_speed_pid.output;
		cloud_motor_output(cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);
		
		if (Timetick1ms%10==0)
		{
			Cloud_Cheet(camera.y, camera.x);
		}
	}
	else
	{
		Cloud_Speed(cloud_pitch_position_pid.output, cloud_yaw_position_pid.output);
	
		cloud_para[0].motor_output=cloud_pitch_speed_pid.output;
		cloud_para[1].motor_output=cloud_yaw_speed_pid.output;
		cloud_motor_output(cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);
		
		if (Timetick1ms%10==0)
		{
			Cloud_Position(pitch_mid, yaw_mid);
		}
	}
//	switch(Timetick2ms)
//	{
//		case 1:

//			get_mpu_data();
//			break;
//		 
//		case 2:
//			cloud_p_v_pid();

//			break;
//		
//		case 3:
//			cloud_y_v_pid();
//			//cloud_motor_output(cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);

//			break;

//		default:			
//			underpan_pid();						//���̵��pid����

//			//underpan_motor_output(underpan_para[0].i_output,underpan_para[1].i_output,
//			//					  underpan_para[2].i_output,underpan_para[3].i_output);
//			Timetick2ms=0;
//			break;
//	}

//	if	(Timetick1ms%8==0){
//		underpan_pid();						//���̵��pid����
//		underpan_motor_output(underpan_para[0].i_output,underpan_para[1].i_output,
//								underpan_para[2].i_output,underpan_para[3].i_output);
//	}
	

	
	if (Timetick1ms%4==0)
	{
		Bodan_pid();
	}
	
	if (Timetick1ms>1000) Timetick1ms=0;
	
	
}
