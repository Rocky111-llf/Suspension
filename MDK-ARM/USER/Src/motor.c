#include "motor.h" 
#include "tim.h"
#include "uartPack.h"
#include "usart.h"
#include "stdio.h"

Motor motor1;
Motor motor2;
extern uart_o_ctrl_t uart_1;

void Motor_Init(void)
{
	//HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim13);                       //开启20ms定时器中断
	//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);            //开启PWM
	__HAL_TIM_SET_COUNTER(&htim1, 30000);                //编码器定时器初始值设定为10000
	motor1.loopNum = 0;
	motor2.loopNum = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static float kp=0;
	static float tspeed=0;
	static float ki=0;
	static float kd=0;
	if(htim->Instance==htim13.Instance)		              //20ms中断
	{
		Uart_O_Timeout_Check(&huart1,&uart_1);
		int16_t pluse1 = COUNTERNUM1 - RELOADVALUE1/2;
		int16_t pluse2 = COUNTERNUM1 - RELOADVALUE1/2;
        //从开始到现在当前50ms的总脉冲数							               									
		motor1.totalAngle = pluse1 + motor1.loopNum * RELOADVALUE1/2;  
		motor2.totalAngle = pluse2 + motor2.loopNum * RELOADVALUE2/2;
        //进行速度计算,根据前文所说的,4倍频,编码器13位,减速比30,再乘以1200即为每分钟输出轴多少转
        //motor.totalAngle - motor.lastAngle为当前10ms内的增量，即脉冲数
		motor1.speed = (float)(motor1.totalAngle - motor1.lastAngle)/(4*13*RR)*3000;
		motor2.speed = (float)(motor2.totalAngle - motor2.lastAngle)/(4*13*RR)*3000;
		printf("speed1:%f\r\n",motor1.speed);//发送速度给串口			
		motor1.lastAngle = motor1.totalAngle;              //更新转过的圈数
		motor2.lastAngle = motor2.totalAngle;
		if(uart_1.rxSaveFlag){
			uart_1.rxSaveFlag=0;
			if(sscanf((char*)uart_1.rxSaveBuf,"kp:%f",&kp)==1){
				printf("!sp:%f\r\n",kp);
				motor1.pid.inner.kp = kp;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"tspeed:%f",&tspeed)==1){
				printf("!tspeed:%f\r\n",tspeed);
				motor1.targetSpeed = tspeed;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"ki:%f",&ki)==1){
				printf("!ki:%f\r\n",ki);
				motor1.pid.inner.ki = ki;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"kd:%f",&kd)==1){
				printf("!kd:%f\r\n",kd);
				motor1.pid.inner.kd = kd;
			}
		}
		
	}
 
    //如果是编码器更新中断,即10ms内,脉冲数超过了计数范围,需要进行处理
	else if(htim->Instance == htim1.Instance)              
	{
		if(COUNTERNUM1 < 30000)	motor1.loopNum++;         //向上计数超过10000，正溢出+1
		else if(COUNTERNUM1 > 30000)	motor1.loopNum--;     //向下计数小于0，负溢出+1
		__HAL_TIM_SetCounter(&htim1, 30000);             //重新设定初始值			
	}else if(htim->Instance == htim3.Instance){
		if(COUNTERNUM2 < 30000)	motor2.loopNum++;         //向上计数超过10000，正溢出+1
		else if(COUNTERNUM2 > 30000)	motor2.loopNum--;     //向下计数小于0，负溢出+1
		__HAL_TIM_SetCounter(&htim3, 30000);             //重新设定初始值		
	}
}
void Motor_Send(void)
{
    PID_SingleCalc(&motor1.pid, motor1.targetSpeed, motor1.speed);      //进行PID计算，传入我们设定的目标值targetSpeed，和我们上篇文章处理编码器值得到的speed值
	PID_SingleCalc(&motor2.pid, motor2.targetSpeed, motor2.speed);
	if(motor1.pid.output > 0)        //对应正转
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)motor1.pid.output);
		IN1(0);
		IN2(1);
	}
	else				  //对应反转					
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)(-motor1.pid.output));
		IN1(1);
		IN2(0);
	}

	if(motor2.pid.output > 0)        //对应正转
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint32_t)motor2.pid.output);
		IN3(0);
		IN4(1);
	}
	else				  //对应反转					
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint32_t)(-motor2.pid.output));
		IN3(1);
		IN4(0);
	}
}
