/**
  *@file usart.c
  *@date 2018-10-11
  *@author Vacuo.W
  *@brief 
  */

#include "my_usart.h"
#include "control.h"
#include <math.h>
#include "pid.h"


//struct CAMERA camera;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

uint8_t cheat_ready=1,cheat_counter;
uint8_t a;
double angle;

uint8_t RX_PID_Count = 0;
uint8_t RX_PID_Buf[20];
uint8_t		RX_PID_Sum = 0;
uint8_t		pidReadBuf;
uint8_t charBuf[4];
PID_Regulator_t 	*pidAdjust;

void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}
uint8_t data[10];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    
	/************小电脑串口数据处理*************/
    if (huart->Instance==USART2)
    {
        switch (camera.count)
        {
            case 0:																//the first data should be '&',or MCU will refuse the data
                if (camera.recieve[0]=='&') camera.count=1;
                else camera.count=0;
								data[camera.count] = camera.recieve[0];
                break;
            case 1:																//the second data should be '%'
                if (camera.recieve[0]=='%') camera.count=2;
                else camera.count=0;
								data[camera.count] = camera.recieve[0];
                break;
            case 2:
                camera.sum = '%'+'&';											//camera.sum will add all data
                camera.x = camera.recieve[0]<<8;								//data of x
                camera.sum += camera.recieve[0];
                camera.count=3;
								data[camera.count] = camera.recieve[0];
                break;
            case 3:
                camera.x += camera.recieve[0];									//data of x
                camera.sum += camera.recieve[0];
                camera.count=4;
								data[camera.count] = camera.recieve[0];
                break;
            case 4:
                if (camera.recieve[0]=='-') camera.x = -camera.x;				//sign of x
                camera.sum += camera.recieve[0];
                camera.count=5;
								data[camera.count] = camera.recieve[0];
                break;
            case 5:
                camera.y = camera.recieve[0]<<8;
                camera.sum += camera.recieve[0];
                camera.count=6;
								data[camera.count] = camera.recieve[0];
                break;
            case 6:
                camera.y += camera.recieve[0];
                camera.sum += camera.recieve[0];
                camera.count=7;
								data[camera.count] = camera.recieve[0];
                break;
            case 7:
                if (camera.recieve[0]=='-') camera.y = -camera.y;
                camera.sum += camera.recieve[0];
                camera.count=8;
								data[camera.count] = camera.recieve[0];
                break;
            case 8:
                if (camera.sum==camera.recieve[0])								//camera.sum should equal this data
                {						
                    //camera.transmit[0]='R';
                    //HAL_UART_Transmit(&huart2,camera.transmit,1,1000);		//response to TX2
										
					if (tele_data.s1==1){
						if (cheat_counter==0){
							if (cheat_ready==1){				
								angle=camera.y;
								pitch=cloud_para[0].Bmechanical_angle+angle;
								angle=atan(camera.x*0.00222)*1304.05;
								yaw=cloud_para[1]. Bmechanical_angle-angle ;
								cheat_ready=0;
							
							}
							if(cloud_para[1].Bmechanical_angle-yaw>-30&&cloud_para[1].Bmechanical_angle-yaw<30)
							if(cloud_para[0].Bmechanical_angle-pitch>-100&&cloud_para[0].Bmechanical_angle-pitch<100)
								cheat_ready=1;
							
						}
						cheat_counter++;
						
						if (cheat_counter==1) cheat_counter=0;					//loss  data,wait for camera
					}
				}
                else {
                    camera.x=0;
                    camera.y=0;
                    camera.transmit[0]='W';
                    HAL_UART_Transmit(&huart2,camera.transmit,1,1000);
                }
                camera.count=0;
                break;
							
				
				

	
					if(pitch<(pitch_mid-600))pitch=pitch_mid-600;
					if(pitch>(pitch_mid+600))pitch=pitch_mid+600;
					if(yaw<(yaw_mid-800))yaw=yaw_mid-800;
					if(yaw>(yaw_mid+800))yaw=yaw_mid+800;
					
		}   
	}
	/******************裁判系统串口数据处理********************/
	else if (huart->Instance==USART6)
	{
		
		
		
		
	}
	
	else if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//是否开始接收
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//接收正确
			if (rxPID.Sum == rxPID.pidReadBuf)
			{
				for (int i = 0; i < 4; i++)
					charBuf[i] = rxPID.Buf[i + 3];

				switch (rxPID.Buf[1])
				{
				case 'p':
					memcpy(&rxPID.pidAdjust->kp, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->kp = -rxPID.pidAdjust->kp;
					break;
				case 'i':
					memcpy(&rxPID.pidAdjust->ki, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->ki = -rxPID.pidAdjust->ki;
					break;
				case 'd':
					memcpy(&rxPID.pidAdjust->kd, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->kd = -rxPID.pidAdjust->kd;
					break;
				}
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
			else
			{
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
		}
		else
			rxPID.Sum += rxPID.pidReadBuf;
	}
    
 
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET) {
	
 __HAL_UART_CLEAR_OREFLAG(huart);
			a=huart->Instance->DR;
	}
		
	
}