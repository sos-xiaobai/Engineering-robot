/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "robotarm_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Robotarm Robotarm;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief 云台初始化
 *
 */
void Class_Robotarm::Init()
{
	//机械臂解算任务初始化
	Robotarm_Resolution.Robotarm = this;
	Robotarm_Resolution.Init(10 , 0);
	//imu初始化
	//Boardc_BMI.Init();
	//底盘通信初始化
	Chassis_Communication.Init(&hcan2);
	//设置机械臂为正常模式
	Set_Robotarm_Control_Type(Robotarm_Control_Type_NORMAL);
    //1轴电机
  	Motor_Joint1.Init(&hcan2, AK_Motor_ID_0x04, CAN_PACKET_SET_POS_SPD , 30.0f , 2.0f);
	//2轴电机
	Motor_Joint2.Init(&hcan1, AK_Motor_ID_0x02, CAN_PACKET_SET_POS_SPD ,30.0f , 2.0f);
    //3轴电机
	Motor_Joint3.Init(&hcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.94359f, 10.0f);
	//4轴电机
	Motor_Joint4.PID_Angle.Init(0.1f, 0.0f, 0.0f, 0.0f, 40.0f, 60.0f);
	Motor_Joint4.PID_Omega.Init(500.0f, 0.0f, 0.0000f, 0, Motor_Joint4.Get_Output_Max(), Motor_Joint4.Get_Output_Max()/10);
	Motor_Joint4.Init(&hcan1, DJI_Motor_ID_0x202,DJI_Motor_Control_Method_ANGLE);
	//5轴电机
	Motor_Joint5.PID_Angle.Init(5.0f, 0.0f, 0.00f, 0.0f, 180.0f, 180.0f);
	Motor_Joint5.PID_Omega.Init(5000.0f, 0.0f, 0.0000f, 0, Motor_Joint5.Get_Output_Max(), Motor_Joint5.Get_Output_Max());
	Motor_Joint5.Init(&hcan1, DJI_Motor_ID_0x203,DJI_Motor_Control_Method_ANGLE);
	
	// for(uint8_t i = 0;i < 5;i++)
	// {
	// 	Joint_Offset_Angle[i] = Joint_Limit_Angle[Joint_Limit_Flag[i]][i];
	// }
}

/**
 * @brief 输出到电机
 *
 */

void Class_Robotarm::Output()
{
    if (Robotarm_Control_Type == Robotarm_Control_Type_DISABLE)
    {
        //云台失能
		for(uint8_t i=0;i<5;i++)
		{
			switch(i)
			{
				case 0:
				case 1:
				{
					auto temp_Motor = reinterpret_cast<Class_AK_Motor_80_6 *>(Motor_Joint[i]);
//					temp_Motor->Set_AK_Motor_Control_Method(CAN_PACKET_DIS_RUN_CONTROL);
				}
				break;
				
				case 2:
					{
						auto temp_Motor = reinterpret_cast<Class_DM_Motor_J4310 *>(Motor_Joint[i]);
						temp_Motor->Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
//						temp_Motor->PID_Angle.Set_Integral_Error(0.0f);
//						temp_Motor->PID_Omega.Set_Integral_Error(0.0f);
//						temp_Motor->Set_Target_Torque(0.0f);
					}
				break;
				case 3:
				case 4:
					{
						auto temp_Motor = reinterpret_cast<Class_DJI_Motor_C610 *>(Motor_Joint[i]);
						temp_Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
						temp_Motor->PID_Angle.Set_Integral_Error(0.0f);
						temp_Motor->PID_Omega.Set_Integral_Error(0.0f);
						temp_Motor->Set_Target_Torque(0.0f);
					}
				break;
			}
		}
    }
    else if (Robotarm_Control_Type == Robotarm_Control_Type_NORMAL)
    {
		if(Robotarm_Resolution.Get_Now_Status_Serial() != Robotarm_Task_Status_Calibration)
        {
			//云台工作
			Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RUN_CONTROL);	
			Motor_Joint2.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RUN_CONTROL);	
			Motor_Joint3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
			Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Motor_Joint5.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			
			
		}
		Set_Motor_Angle(Motor_Joint1,1);
		Set_Motor_Angle(Motor_Joint2,2);
		Set_Motor_Angle(Motor_Joint3,3);
		Set_Motor_Angle(Motor_Joint4,4);
		Set_Motor_Angle(Motor_Joint5,5);
		
    }
}
void Class_Robotarm::Task_Alive_PeriodElapsedCallback()
{
	memcpy(&Custom_Communication_Data,&Referee.Interaction_Custom_Controller,sizeof(Struct_Custom_Communication_Data));
	static bool Suspend_flag = false;
	static bool alive_flag = false;
	if(alive_flag == false)
	{
		Motor_Joint1.Task_Alive_PeriodElapsedCallback();
		Motor_Joint2.Task_Alive_PeriodElapsedCallback();
		alive_flag = true;
	}
	else
	{
		Motor_Joint3.TIM_Alive_PeriodElapsedCallback();
		alive_flag = false;
	}
	
	 Motor_Joint4.TIM_Alive_PeriodElapsedCallback();
	 Motor_Joint5.TIM_Alive_PeriodElapsedCallback();
	
	if((Motor_Joint1.Get_AK_Motor_Status() == AK_Motor_Status_ENABLE)
	&& (Motor_Joint2.Get_AK_Motor_Status() == AK_Motor_Status_ENABLE)
	&& (Motor_Joint3.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE)
//	&& (Motor_Joint4.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
//	&& (Motor_Joint5.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
	&& (Suspend_flag == true))
	{
		Robotarm_Control_Type = Robotarm_Control_Type_NORMAL;
		//osThreadResume(RobotarmTaskHandle);
		Suspend_flag = false;
	}
	else if(((Motor_Joint1.Get_AK_Motor_Status() == AK_Motor_Status_DISABLE)
	|| (Motor_Joint2.Get_AK_Motor_Status() == AK_Motor_Status_DISABLE)
	|| (Motor_Joint3.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE))
//	|| (Motor_Joint4.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
//	|| (Motor_Joint5.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE))
	&& (Suspend_flag == false))
	{
	    Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
		Robotarm_Resolution.Set_Status(Robotarm_Task_Status_Calibration);
		for(uint8_t i = 0;i < 5;i++)
		{
			Joint_Offset_Angle[i] = Joint_Limit_Angle[Joint_Limit_Flag[i]][i];
			Joint_World_Angle[i] = 0;
		}
		Motor_Joint1.Reset_Rx_Data();
		Motor_Joint2.Reset_Rx_Data();
		Motor_Joint1.Set_Target_Angle(0);
		Motor_Joint2.Set_Target_Angle(0);
		Motor_Joint3.Set_Target_Angle(0);
		Target_Position_Orientation = {209.314f, 0 ,30.0f};
		//osThreadSuspend(RobotarmTaskHandle);//挂起任务
		Suspend_flag = true;
	}
}

/**
 * @brief Task计算回调函数
 *
 */
void Class_Robotarm::Task_Calculate_PeriodElapsedCallback()
{
    Output();
	
	Motor_Joint1.Task_Process_PeriodElapsedCallback();
	Motor_Joint2.Task_Process_PeriodElapsedCallback();
	Motor_Joint3.TIM_Process_PeriodElapsedCallback();
	Motor_Joint4.TIM_PID_PeriodElapsedCallback();
	Motor_Joint5.TIM_PID_PeriodElapsedCallback();
}

/**
 * @brief 遥控器控制任务
 *
 */
void Class_Robotarm::Task_Control_Robotarm()
{
	//角度目标值
//    float tmp_robotarm_x, tmp_robotarm_y,robotarm_yaw;
	//遥控器摇杆值
	memcpy(&Custom_Communication_Data,&Referee.Interaction_Custom_Controller,sizeof(Struct_Custom_Communication_Data));
	static Position_Orientation_t Last_Position_Orientation = Target_Position_Orientation;
	static Chassis_Move_t Last_Chassis_Move = Chassis_Move;
	
	if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
	{
		float dr16_left_x, dr16_left_y,dr16_right_x,dr16_right_y,dr16_yaw;
		// 排除遥控器死区
		dr16_left_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
		dr16_left_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
		dr16_right_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
		dr16_right_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
		dr16_yaw = (Math_Abs(DR16.Get_Yaw()) > DR16_Dead_Zone) ? DR16.Get_Yaw() : 0;
		switch(DR16.Get_Left_Switch())
		{
			case DR16_Switch_Status_UP:
				//底盘移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
						Target_Position_Orientation.Z_Position += dr16_right_y * Robotarm_Z_Resolution;
						Chassis_Move.Chassis_Vx = dr16_left_x * Chassis_X_Resolution;
						Chassis_Move.Chassis_Vy = dr16_left_y * Chassis_Y_Resolution;
						Chassis_Move.Chassis_Wz = dr16_right_x * Chassis_Z_Resolution;
					break;
					case DR16_Switch_Status_DOWN:
						
					break;
				}
				if((Last_Position_Orientation != Target_Position_Orientation)||(Last_Chassis_Move != Chassis_Move))
				{
					//osSemaphoreRelease(Communication_SemHandle);
					Last_Position_Orientation = Target_Position_Orientation;
					Last_Chassis_Move = Chassis_Move;
				}
				
			break;
			case DR16_Switch_Status_MIDDLE:
				//机械臂移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
						
					break;
					case DR16_Switch_Status_MIDDLE:
						Target_Position_Orientation.X_Position += dr16_left_y * Robotarm_X_Resolution;
						Target_Position_Orientation.Y_Position -= dr16_left_x * Robotarm_Y_Resolution;
						
						Target_Position_Orientation.Pitch_Angle -= dr16_right_y * Robotarm_Pitch_Resolution;
						Target_Position_Orientation.Roll_Angle += dr16_right_x * Robotarm_Roll_Resolution;
						Target_Position_Orientation.Yaw_Angle += dr16_yaw * Robotarm_Yaw_Resolution;
					break;
					case DR16_Switch_Status_DOWN:
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
					break;
				}
				
			break;
			case DR16_Switch_Status_DOWN:
				//整车无力
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
//						Target_Position_Orientation = {209.314f, 0 ,30.0f};
					break;
					case DR16_Switch_Status_DOWN:
						Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
					break;
				}
				
			break;
		}
	}
	if(Referee.Get_Referee_Status() == Referee_Status_ENABLE)
	{
		float Controller_x, Controller_y,Controller_pitch,Controller_roll,Controller_yaw;
		
		Controller_x = Math_Int_To_Float(Custom_Communication_Data.Flow_x, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_y = Math_Int_To_Float(Custom_Communication_Data.Flow_y, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_pitch = Math_Int_To_Float(Custom_Communication_Data.pitch, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_roll = Math_Int_To_Float(Custom_Communication_Data.roll, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_yaw = Math_Int_To_Float(Custom_Communication_Data.yaw, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		
		Controller_x = (Math_Abs(Controller_x) > Controller_Dead_Zone) ? Controller_x : 0;
		Controller_y = (Math_Abs(Controller_y) > Controller_Dead_Zone) ? Controller_y : 0;
		Controller_pitch = (Math_Abs(Controller_pitch) > Controller_Dead_Zone) ? Controller_pitch : 0;
		Controller_roll = (Math_Abs(Controller_roll) > Controller_Dead_Zone) ? Controller_roll : 0;
		Controller_yaw = (Math_Abs(Controller_yaw) > Controller_Dead_Zone) ? Controller_yaw : 0;
		
		Target_Position_Orientation.X_Position -= (float)Controller_x * Robotarm_X_Resolution;
		Target_Position_Orientation.Y_Position -= (float)Controller_y * Robotarm_Y_Resolution ;
		
		Target_Position_Orientation.Pitch_Angle -= Controller_pitch * Robotarm_Pitch_Resolution;
		Target_Position_Orientation.Roll_Angle -= Controller_roll * Robotarm_Roll_Resolution;
		Target_Position_Orientation.Yaw_Angle += Controller_yaw * Robotarm_Yaw_Resolution;
	}
//	else
//	{
//		Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
//	}

}

/**
 * @brief 底盘通信任务，当遥控器值更新时才发送
 *

 */
void Class_Robotarm::Task_Chassis_Communication_PeriodElapsedCallback()
{
	//同步信号量等待
	//osSemaphoreWait(Communication_SemHandle,osWaitForever);
	//底盘速度内容填充
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vx, -1.0f, 1.0f , Chassis_Communication_ID_0x11,0);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vy, -1.0f, 1.0f , Chassis_Communication_ID_0x11,2);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Wz, -1.0f, 1.0f , Chassis_Communication_ID_0x11,4);
	//机械臂抬升高度
	Math_Constrain(Target_Position_Orientation.Z_Position,0.0f,1160.0f);
	Chassis_Communication.Communication_Data(Target_Position_Orientation.Z_Position, 0, 1160.0f , Chassis_Communication_ID_0x11,6);
	
	
	//发送函数
	Chassis_Communication.Task_Process_PeriodElapsedCallback();
}

/**
 * @brief C620校准函数
 *
 * @param Motor C620指针
 * @param num 关节角标
 * @param Cali_Omega 校准速度
 * @param Cali_Max_Out 校准最大电流输出
 * @param Target_Angle 关节目标角度
 */
bool Class_Robotarm::Motor_Calibration(Class_DJI_Motor_C620 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle)
{
	//记录电机堵转时间
	static uint16_t count[2] = {0,0};
	//设置为速度环校准
	Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Motor.Set_Target_Omega_Radian(Cali_Omega);
	Motor.PID_Omega.Set_Out_Max(Cali_Max_Out);
	
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if((fabs(Motor.Get_Now_Torque()) >= 1500) && (fabs(Motor.Get_Now_Omega_Radian()) < 0.01f*PI))
	{
		count[num - 3]++;
		//当到达一定时间，判定为堵转
		if(count[num - 3] >= 200)
		{
			count[num - 3] = 0;
			//改为角度环，设置关节角度
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Joint_World_Angle[num - 1] = Target_Angle;
			Joint_Offset_Angle[num - 1] = Motor.Get_Now_Angle();
			Motor.PID_Omega.Set_Out_Max(16384);
			return true;
		}
	}
	return false;
}
/**
 * @brief AK校准函数
 *
 * @param Motor AK80_6指针
 * @param num 关节角标 1为1轴 2为2轴
 * @param Cali_Max_Out 校准最大电流输出
 * @param Cali_Omega 校准途中的转速 电气转速
 * @param Target_Angle 关节目标角度
 */
bool Class_Robotarm::Motor_Calibration(Class_AK_Motor_80_6 &Motor,uint8_t num,float Cali_Omega,float Target_Angle)
{
	//记录电机堵转时间
	// if(Motor.Get_Rx_Data() != 0)
	// {
	// 	if(count[num - 1] == 0)
	// 	{
	// 		Motor.Slope_Joint_Angle.Set_Increase_Value(180);
	// 		Motor.Slope_Joint_Angle.Set_Decrease_Value(180);
	// 		Joint_Offset_Angle[num - 1] += Motor.Get_Now_Angle();
	// 		count[num - 1]++;
	// 	}
	// 	else
	// 	{
	// 		//角度增量
	// 		Joint_Offset_Angle[num - 1] += Cali_Omega;
	// 		if((fabs(Motor.Get_Now_Torque()) >= 5.0f))
	// 		{
	// 			count[num - 1]++;
	// 			if(count[num - 1] >= 50)
	// 			{
	// 				Motor.Slope_Joint_Angle.Set_Increase_Value(0.1);
	// 				Motor.Slope_Joint_Angle.Set_Decrease_Value(0.1);
	// 				count[num - 1] = 0;
	// 				Joint_Offset_Angle[num - 1] = Motor.Get_Now_Angle();
	// 				Joint_World_Angle[num - 1] = Target_Angle;
	// 				return true;
	// 			}
	// 		}
	// 	}
	// }
	// return false;
	static uint16_t count[2] = {0,0};
	Motor.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RPM);
	if(Motor.Get_Rx_Data() != 0)
	{
		Motor.Set_Target_Omega(Cali_Omega);
		if(fabs(Motor.Get_Now_Torque()) >= 5.0f)
		{
			count[num - 1]++;
			if(count[num - 1] >= 50)
			{
				//记录当前限位角度
				Joint_Offset_Angle[num - 1] = Motor.Get_Now_Angle();
				//改为速度角度控制，设置当前角度为目标角度
				// Motor.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
				// Motor.Set_Target_Omega(Motor.get_Max_Omega());
				// Motor.Set_Target_Angle(Joint_Offset_Angle[num - 1]);
				Motor.Set_Target_Omega(0);
				Motor.Task_Process_PeriodElapsedCallback();
				//清零计数
				count[num - 1] = 0;
				//返回校准成功
				return true;
			}
		}
	}
	return false;
}
/**
 * @brief C610校准函数
 *
 * @param Motor C610指针
 * @param num 关节角标
 * @param Cali_Omega 校准速度
 * @param Cali_Max_Out 校准最大电流输出
 * @param Target_Angle 关节目标角度
 */
bool Class_Robotarm::Motor_Calibration(Class_DJI_Motor_C610 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle)
{
	//记录电机堵转时间
	static uint16_t count;
	//设置为速度环校准
	Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Motor.Set_Target_Omega_Radian(Cali_Omega);
	Motor.PID_Omega.Set_Out_Max(Cali_Max_Out);
	
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Motor.Get_Now_Torque()) >= 1000) && (fabs(Motor.Get_Now_Omega_Radian()) < 0.01*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环，设置关节角度
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Joint_World_Angle[num-1] = Target_Angle;
			Joint_Offset_Angle[num-1] = Motor.Get_Now_Angle();
			Motor.PID_Omega.Set_Out_Max(10000);
			return true;
		}	
	}
	return false;
}

/**
 * @brief 机械臂校准任务
 *
 */

bool Class_Robotarm::Robotarm_Calibration()
{
    //电机校准标志位，1位表示一个电机
	//关节1校准
	static uint8_t Arm_Cal_Flag =0;
	if((Arm_Cal_Flag & (1<<1)) == 0)
	{
		if(Motor_Calibration(Motor_Joint1,1,300.0f,85.422f) == true)
		{
			Arm_Cal_Flag |= (1<<1);
		}
	}
	//关节2校准
	if((Arm_Cal_Flag & (1<<2)) == 0)
	{
		if(Motor_Calibration(Motor_Joint2,2,300.0f,-170.845f) == true)
		{
			Arm_Cal_Flag |= (1<<2);
		}
	}

	// //关节4校准
	// if((Arm_Cal_Flag & (1<<4)) == 0)
	// {
	// 	if(Motor_Calibration(Motor_Joint4,4,0.2*PI,4000,0) == true)
	// 	{
	// 		Arm_Cal_Flag |= (1<<4);
	// 	}
	// }
	// //关节5校准
	// if((Arm_Cal_Flag & (1<<5)) == 0)
	// {
	// 	if(Motor_Calibration(Motor_Joint5,5,-0.2*PI,3000,0) == true)
	// 	{
	// 		Arm_Cal_Flag |= (1<<5);
	// 	}
	// }
	
	//校准完毕后，标志位全部置1
	// if(Arm_Cal_Flag == 54)
	// {
	// 	Arm_Cal_Flag = 0;
	// 	return true;
	// }
	// else
	// {
	// 	return false;
	// }
	if(Arm_Cal_Flag == 6)
	{
		Arm_Cal_Flag = 0;
		return true;
	}
	else
	{
		return false;
	}

}
/**
 * @brief 机械臂解算回调函数
 *
 */
void Class_Robotarm_Resolution::Reload_Task_Status_PeriodElapsedCallback()
{
	float Beta = 0.0f,Joint1_Angle_1 = 0.0f,Joint1_Angle_2 = 0.0f,Joint2_Angle = 0.0f,Joint3_Angle = 0.0f;
	static Position_Orientation_t Last_Correct_Position_Orientation = Robotarm->Target_Position_Orientation;
	if(Robotarm->Robotarm_Control_Type == Robotarm_Control_Type_NORMAL)
	{
		Status[Now_Status_Serial].Time++;
		//自己接着编写状态转移函数
		switch (Now_Status_Serial)
		{
			//校准任务
			case (Robotarm_Task_Status_Calibration):
			{
				//电机校准完成后进入下一转态
				if(Robotarm->Robotarm_Calibration() == true)
				{
					Set_Status(Robotarm_Task_Status_Resolution);
				}
			}
			break;
			case (Robotarm_Task_Status_Resolution):
			{

			}
			case (Robotarm_Task_Status_decision):
			{
				Set_Status(Robotarm_Task_Status_Resolution);
			}
			break;
			case (4):
			{

			}
			break;
		}
	}
}
 /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
