#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))  //限幅定义

typedef struct _PID
{
	float kp,ki,kd;               //PID的三个参数
	float error,lastError;        //当前误差和上一次的误差
	float integral,maxIntegral;   //积分量和积分的限幅
	float output,maxOutput;       //PID的输出量和PID的最大输出量
}PID;

void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut);
void PID_SingleCalc(PID *pid,float reference,float feedback);
#endif
