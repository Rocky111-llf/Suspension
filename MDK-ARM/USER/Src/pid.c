#include "pid.h" 
#include "math.h"

extern double abs(double __x);
//PID初始化函数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
}

//单级PID计算，需要传入的参数，PID结构体，目标值，反馈值
void PID_SingleCalc(PID *pid,float reference,float feedback)
{ 
	static uint16_t inl_index = 0;
	pid->error = reference-feedback;                          //更新当前误差
    //下面分别是P，I，D的计算
    pid->output = pid->error*pid->kp;                        //P为根据当前误差计算输出量
	if(abs(pid->error) > 200.0){
		inl_index = 0;
	}else{
		inl_index = 1;
		pid->integral += pid->error*pid->ki;                       //I为累计误差的输出量
	}
    // LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);  //限制I的输出，抑制超调
    pid->output += inl_index * pid->integral;   
	pid->output += (pid->error - pid->lastError)*pid->kd;      //D以当前误差减去上次误差作为微分环节
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);        //限制PID总输出
	pid->lastError = pid->error;                              //更新上一次的误差
}
//串级PID计算
void PID_AngleCalc(AnglePID *pid, float target_angle, int32_t total_angle, uint32_t speed){
	PID_SingleCalc(&pid->outer, target_angle, total_angle);
	PID_SingleCalc(&pid->inner, pid->outer.output, speed);
	pid->output = pid->inner.output;
}
