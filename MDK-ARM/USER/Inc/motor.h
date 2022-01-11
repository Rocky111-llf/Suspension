#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f4xx.h"
#include "main.h"
#include "pid.h"


#define RR 30u    //电机减速比
#define RELOADVALUE1 __HAL_TIM_GetAutoreload(&htim1)    //获取自动装载值,本例中为20000
#define COUNTERNUM1 __HAL_TIM_GetCounter(&htim1)        //获取编码器定时器中的计数值
#define RELOADVALUE2 __HAL_TIM_GetAutoreload(&htim3)    //获取自动装载值,本例中为20000
#define COUNTERNUM2 __HAL_TIM_GetCounter(&htim3)        //获取编码器定时器中的计数值
#define IN1(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,(GPIO_PinState)(state))    //M1
#define IN2(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,(GPIO_PinState)(state))    //M2
#define IN3(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)(state))    //M3
#define IN4(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,(GPIO_PinState)(state))    //M4

typedef struct _AnglePID
{
	PID inner;
	PID outer;
	uint32_t output;

}AnglePID;

typedef struct _Motor
{
	int32_t lastAngle;        //上10ms转过的角度
	int32_t totalAngle;       //总的角度
	uint32_t targetAngle;     //目标角度
	int16_t loopNum;          //溢出次数计数值
	float speed;              //电机输出轴目前转速,单位为RPM
	float targetSpeed;       //添加设定的目标速度
	AnglePID pid;                 //添加电机对应PID
}Motor;





extern Motor motor1;
extern Motor motor2;

void Motor_Init(void);
void Motor_Send(void);


#endif
