/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "CharSendTask.h"
#include "GraphicsSendTask.h"
#include "drv_usb.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "config.h"
//#include "GraphicsSendTask.h"
//#include "ui.h"
#include "dvc_GraphicsSendTask.h"
#include "robotarm_task.h"
#include "dvc_message.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern Class_Robotarm Robotarm;

extern Struct_USB_Manage_Object MiniPC_USB_Manage_Object;

//注册发布者
Publisher Joint5_Sub_Angle = Message_Manager.PubRegister("Joint5_angle", sizeof(float));

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
	}
	
    switch (temp_id)
    {
        case (0x14):
        {
            
        }
        break;
        case (0x12):
        {
            Robotarm.Motor_Joint2.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x05):
        {
          Robotarm.Motor_Joint3.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
          Robotarm.Motor_Joint4.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
          Robotarm.Motor_Joint5.CAN_RxCpltCallback(CAN_RxMessage->Data);
          float tmp_angle = Robotarm.Motor_Joint5.Get_Now_Angle();
          Message_Manager.PubPush_Message<float>(Joint5_Sub_Angle, tmp_angle);
        }
        break;
    }
}


void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
	}
	
    switch (temp_id)
    {
        case 0x014:
        {
            Robotarm.Motor_Joint1.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case 0x02:
        {
            
        }
        break;
        case (0x05):
        {
            
        }
        break;
        case (0x202):
        {
            
        }
        break;
        case (0x203):
        {
            
        }
        break;
    }
}



/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}

/**
 * @brief UART1图传回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.DR16.Image_UART_RxCpltCallback(Buffer);
}



/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.DR16.DR16_UART_RxCpltCallback(Buffer);
}


/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */

void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    Robotarm.Referee.UART_RxCpltCallback(Buffer, Length);
}


/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */

void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
    // 单给IMU消息开的定时器 ims
    // Robotarm.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();     
}



/**
 * @brief TIM5任务回调函数
 *
 */

//注册订阅者
Subscriber Task_Sub_Joint5_angle = Message_Manager.SubRegister("Joint5_angle", sizeof(float));


float m2006_test_angle_before = 0;
float m2006_test_angle_after = 0;

float m3508_test_angle_before = 0;
float m3508_test_angle_after = 0;

float joint5_test_angle = 0;

float dm_test_angle_before = 180;
float dm_test_angle_after = 0;
float dm_test_omega = 1;

float Ak_2_test_angle_before = 180;
float Ak_2_test_angle_after = 0;
float Ak_2_test_omega = 0;

float Ak_1_test_angle_before = 0;
float Ak_1_test_angle_after = 0;
float Ak_1_test_omega = 0;

uint8_t m2006_test =1;
uint8_t AK_Motor_CAN_Message_Save_Zero_1[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

bool init_finished = false;

void Task1ms_TIM5_Callback()
{

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    static uint8_t TIM1msMod50 = 0;
    TIM1msMod50++;
    if(TIM1msMod50 == 50 )
    {
        TIM1msMod50 = 0;
        if(!init_finished)
        {
            Robotarm.Motor_Joint1.Task_Alive_PeriodElapsedCallback();
            Robotarm.Motor_Joint2.Task_Alive_PeriodElapsedCallback();            
        }
        Robotarm.Motor_Joint3.TIM_Alive_PeriodElapsedCallback();
        Robotarm.Motor_Joint4.TIM_Alive_PeriodElapsedCallback();
	    Robotarm.Motor_Joint5.TIM_Alive_PeriodElapsedCallback();
        // Robotarm.Task_Alive_PeriodElapsedCallback();
    }
 //CAN_Send_Data(&hcan1, 0x14, AK_Motor_CAN_Message_Save_Zero_1, (uint16_t)8,CAN_ID_STD);
    //Robotarm.Motor_Joint1.Task_PID_PeriodElapsedCallback();
    // Ak_1_test_angle_after = Ak_1_test_angle_before - 270.0f;

    if(!init_finished)
    {
        init_finished = Robotarm.Robotarm_Calibration();
    }
    if(init_finished)
    {
        Robotarm.Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
        Ak_1_test_angle_after = Ak_1_test_angle_before + Robotarm.Get_Joint_Offset_Angle(1);
        Robotarm.Motor_Joint1.Set_Target_Omega(Robotarm.Motor_Joint1.get_Max_Omega());
        Robotarm.Motor_Joint1.Set_Target_Angle(Ak_1_test_angle_after);   

        Robotarm.Motor_Joint2.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
        Ak_2_test_angle_after = Ak_2_test_angle_before + (Robotarm.Get_Joint_Offset_Angle(2)-225.0f);
        Robotarm.Motor_Joint2.Set_Target_Omega(Robotarm.Motor_Joint2.get_Max_Omega());
        Robotarm.Motor_Joint2.Set_Target_Angle(Ak_2_test_angle_after);  

        dm_test_angle_after = (dm_test_angle_before - 90.0f) * DEG_TO_RAD;
        Robotarm.Motor_Joint3.Set_Target_Angle(dm_test_angle_after);
        Robotarm.Motor_Joint3.Set_Target_Omega(dm_test_omega);
        Robotarm.Motor_Joint3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        
    }    
    if(m2006_test)
    {
        //m2006_test = 0;
        // Robotarm.Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RPM);
        // Robotarm.Motor_Joint1.Set_Target_Angle(Ak_1_test_angle_after);
        // Robotarm.Motor_Joint1.Set_Target_Omega(Ak_1_test_omega);
        Robotarm.Motor_Joint1.Task_Process_PeriodElapsedCallback();
        Robotarm.Motor_Joint2.Task_Process_PeriodElapsedCallback();
        Robotarm.Motor_Joint3.TIM_Process_PeriodElapsedCallback();  
    }
      

//    m2006_test_angle_after = m2006_test_angle_before + 1.4302572f;
//    Robotarm.Motor_Joint5.Set_Target_Angle(m2006_test_angle_after);
//    Robotarm.Motor_Joint5.TIM_PID_PeriodElapsedCallback();

//    m3508_test_angle_after = m3508_test_angle_before + 14.9686575f;
//    Robotarm.Motor_Joint4.Set_Target_Angle(m3508_test_angle_after);
   Robotarm.Motor_Joint4.TIM_PID_PeriodElapsedCallback();
    if(m2006_test)
    // TIM_CAN_PeriodElapsedCallback();

    //订阅者获取消息
    Message_Manager.SubGet_Message<float>(Task_Sub_Joint5_angle, joint5_test_angle);
   //Robotarm.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

    /****************************** 交互层回调函数 1ms *****************************************/
        
    /****************************** 驱动层回调函数 1ms *****************************************/ 
    //统一打包发送        
//    TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);
    // if((Robotarm.Get_Joint_Offset_Angle(1) != Robotarm.Get_Joint_Limit_Angle(1)) && (Robotarm.Get_Joint_Offset_Angle(2) != Robotarm.Get_Joint_Limit_Angle(2))
    //     &&((Robotarm.Robotarm_Resolution.Status[Robotarm_Task_Status_Calibration].Time !=0)
    //     ||(Robotarm.Robotarm_Resolution.Get_Now_Status_Serial() == Robotarm_Task_Status_Resolution)
    //     ||(Robotarm.Robotarm_Resolution.Get_Now_Status_Serial() == Robotarm_Task_Status_decision)))
    // {
    //     TIM_CAN_PeriodElapsedCallback();
    // }
}

/**
 * @brief 初始化任务
 *
 */

extern "C" void Task_Init()
{  

    DWT_Init(168);
    USB_Init(&MiniPC_USB_Manage_Object, MiniPC_USB_Callback);
    /********************************** 驱动层初始化 **********************************/

    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);
 
    //c板陀螺仪spi外设
    // SPI_Init(&hspi1,Device_SPI1_Callback);

    //磁力计iic外设
    // IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    
  
    //裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128);   //并未使用环形队列 尽量给长范围增加检索时间 减少丢包
    //遥控器接收
    UART_Init(&huart3, DR16_UART3_Callback, 18);
  
    UART_Init(&huart1, Image_UART1_Callback, 40);

    //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    Robotarm.Init();

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    uint8_t DM_Motor_CAN_Message_Enter_1[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    //CAN_Send_Data(&hcan1, 0x101, DM_Motor_CAN_Message_Save_Zero_1, 8,CAN_ID_STD);
    // for(auto i=0;i<50;i++)
    // CAN_Send_Data(&hcan1, 0x101, DM_Motor_CAN_Message_Enter_1, 8,CAN_ID_STD);
    
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
