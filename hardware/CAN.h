#ifndef __CAN_H_
#define __CAN_H_

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_can.h"
#include "stdio.h"
#include "delay.h"
#include "LED.h"
#include "board.h"
#include "SBUS_UART.h"
#include "IO_10_IN.h"

/*keya驱动器相关配置*/
//电机1的扩展ID
#define		Motor1ExtId				(u32)0x06000001
//发送数据的长度
#define		Motor1DLC				(u8)8
//扩展ID模式
#define		Motor1IDE				(u32)CAN_Id_Extended
//数据帧模式
#define		Motor1RTR				(u32)CAN_RTR_Data
//标准ID，可以不设置本电机使用的是扩展ID
#define		Motor1StdId				(u16)0x00


/*阿波罗机器人电机驱动器相关配置*/
//左轮子的扩展ID 可以不设置本电机使用的是标准ID
#define		WheelRightExtId				(u32)0x00000000
//右轮子的扩展ID 可以不设置本电机使用的是标准ID
#define		WheelLeftExtId				(u32)0x00000000
//两个轮子的扩展ID 可以不设置本电机使用的是标准ID
#define		WheelAllExtId				(u32)0x00000000
//发送数据的长度
#define		WheelDLC					(u8)8
//标准ID模式
#define		WheelIDE					(u32)CAN_Id_Standard
//数据帧模式
#define		WheelRTR					(u32)CAN_RTR_Data
//左右轮子的标准ID
#define		WheelLeftStdId				(u16)0x02		//左轮
#define		WheelRightStdId				(u16)0x01		//右轮
//轮子广播标准ID
#define		WheelAllStdId				(u16)0x00



//初始化GPIO,配置CAN
void Init_CAN1_GPIO(void);
//初始化CAN的接收中断
void Init_CAN1_RecvIT(void);
//初始化筛选器
void Init_CAN1_Filter(void);
//初始化CAN（总）
void Init_CAN1();
//配置并发送数据
void CAN1_SetTransmit(CanTxMsg* TxMessage,const char *data,u32 rtr,u32 ide,u16 std_id,u32 ext_id,u8 dlc);
//打印CAN接收的数据
void CAN1_PrintRecvData(CanRxMsg *RxMessage);
//打印CAN发送的数据
void CAN1_PrintSendData(CanTxMsg *TxMessage);
//电机速度控制
void MotorSpeedControl(u16 speed);
//电机使能
void MotorEnable(void);
//电机速度设置
void MotorSpeed(void);
//电机速度设置与正反转选择
void MotorSpeedSet(u16 speed);

static void motor1_thread_entry(void *parameter);

int bsp_motor1_thread(u16 Speed);
void MotorTorqueControl(u16 torque);
void MotorTorqueSet(u16 torque);



//单个电机使能
int8_t WheelEnable(u8 num);
int bsp_Sbus_thread(u8 choose);
static void Sbus_thread_entry(void *parameter);
//发送can数据包
u8 SendCanDataPage(CanTxMsg* TxMessage,const char *data,u32 rtr,u32 ide,u16 std_id,u32 ext_id,u8 dlc);
//单个电机默认速度设置
void SetDefaultSpeed(u8 num);

void WheelEnableAll(void);
//电机全部停转
void SetAllWheelStop(void);
//电机设置速度
void WheelSpeedSet(SBUS_CH_Struct *SBUS_CH);
//单个电机停止
void SetWheelStop(u8 num);
//配置获取小车左右轮的速度
void coordinate_transformation(double x, double y,int *speed,int driver_max_value);//坐标转换
//小车可带转的行走
void TowMotorCarRun(int *speed);

//四电机小车行走
void FourMotorRun(int *speed);
#endif


