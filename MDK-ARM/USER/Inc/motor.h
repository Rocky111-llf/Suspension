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
#define POINT_SPACING 30   //直线运动时到目标中间点的个数
#define METER_CYCLE 0.158     //电机一转对应的米数 

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

typedef struct _Line
{
	uint32_t x;
	uint32_t y;
	uint32_t line_left;
	uint32_t line_right;
	uint32_t line_dl;
	uint32_t line_dr;
}Line;



extern Motor motor1;
extern Motor motor2;
extern Line line;

void Motor_Init(uint32_t x, uint32_t y, uint32_t le, uint32_t ri);
uint8_t Motor_Send(void);
void Speed_Tset1(void);
void Speed_Tset2(void);
uint8_t Line_Control(uint32_t x, uint32_t y);


#endif
