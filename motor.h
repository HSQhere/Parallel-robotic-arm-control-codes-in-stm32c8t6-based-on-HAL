#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <math.h>
// #include "tim.h"


// 机械臂物理参数
#define L1 140.0f
#define L2 130.0f
#define PI 3.14159265f

// 初始位置
#define Init_X 20.0f
#define Init_Y 0.0f
#define Init_Z 20.0f // 目标高度
// (-90, 45, 30)；(-75, 0, 30)；(-90, -45, 30)

// 舵机通道映射 (根据你的实际接线修改)
#define PWM_TIM htim4
#define CH_UPPER TIM_CHANNEL_1 //大臂旋转PB6
#define CH_LOWER TIM_CHANNEL_2 //小臂旋转PB7
#define CH_YAW   TIM_CHANNEL_3 //底座旋转PB8
#define CH_CLAW  TIM_CHANNEL_4 //爪子开合PB9

// 函数声明
void Motor_Init(void);
void Motor_Update_Space_IK(float x, float y, float z);
void Motor_Apply_Hardware_Mapping(float alpha_deg, float beta_deg);
void Motor_SetYaw(float angle);
void Motor_SetClaw(uint8_t open);

#define MOTION_PERIOD     20      // 平滑控制周期（ms），需被主循环定时调用
extern volatile uint8_t MOTOR_MOVE_COMPLETE; // 运动完成标志
void Motor_MoveTo(float x, float y, float z); // 启动平滑运动
void Motor_Process(void);                     // 每 MOTION_PERIOD ms 调用一次

#endif