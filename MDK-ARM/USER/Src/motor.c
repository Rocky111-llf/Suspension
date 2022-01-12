#include "motor.h" 
#include "tim.h"
#include "uartPack.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"

Motor motor1;
Motor motor2;
Line line;
extern uart_o_ctrl_t uart_1;
extern double abs(double __x);

void Motor_Init(uint32_t x, uint32_t y, uint32_t le, uint32_t ri)
{
	//HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim13);                       //开启20ms定时器中断
	//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);            //开启PWM
	__HAL_TIM_SET_COUNTER(&htim1, 30000);                //编码器定时器初始值设定为10000
	motor1.loopNum = 0;
	motor2.loopNum = 0;
	line.x = x;
	line.y = y;
	line.line_left = le;
	line.line_right = ri;
	motor1.pid.inner.integral = 0;
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
		int16_t pluse2 = COUNTERNUM2 - RELOADVALUE2/2;
        //从开始到现在当前50ms的总脉冲数							               									
		motor1.totalAngle = pluse1 + motor1.loopNum * RELOADVALUE1/2;  
		motor2.totalAngle = pluse2 + motor2.loopNum * RELOADVALUE2/2;
        //进行速度计算,根据前文所说的,4倍频,编码器13位,减速比30,再乘以1200即为每分钟输出轴多少转
        //motor.totalAngle - motor.lastAngle为当前10ms内的增量，即脉冲数
		motor1.speed = (float)(motor1.totalAngle - motor1.lastAngle)/(4*13*RR)*3000;
		motor2.speed = (float)(motor2.totalAngle - motor2.lastAngle)/(4*13*RR)*3000;
		// printf("speed1:%f\r\n",motor1.speed);//发送速度给串口
		printf("speed2:%f\r\n",motor2.speed);//发送速度给串口			
		motor1.lastAngle = motor1.totalAngle;              //更新转过的圈数
		motor2.lastAngle = motor2.totalAngle;
		if(uart_1.rxSaveFlag){
			uart_1.rxSaveFlag=0;
			if(sscanf((char*)uart_1.rxSaveBuf,"kp:%f",&kp)==1){
				printf("!kp:%f\r\n",kp);
				// motor1.pid.inner.kp = kp;
				motor2.pid.inner.kp = kp;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"tspeed:%f",&tspeed)==1){
				printf("!tspeed:%f\r\n",tspeed);
				// motor1.targetSpeed = tspeed;
				motor2.targetSpeed = tspeed;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"ki:%f",&ki)==1){
				printf("!ki:%f\r\n",ki);
				// motor1.pid.inner.ki = ki;
				motor2.pid.inner.ki = ki;
			}else if(sscanf((char*)uart_1.rxSaveBuf,"kd:%f",&kd)==1){
				printf("!kd:%f\r\n",kd);
				// motor1.pid.inner.kd = kd;
				motor2.pid.inner.kd = kd;
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
void Speed_Tset1(void){//调电机1速度PID
	PID_SingleCalc(&motor1.pid.inner, motor1.targetSpeed, motor1.speed);
	if(motor1.pid.inner.output > 0)        //对应正转
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)motor1.pid.inner.output);
		IN1(1);
		IN2(0);
	}
	else				  //对应反转					
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)(-motor1.pid.inner.output));
		IN1(0);
		IN2(1);
	}
}
void Speed_Tset2(void){//调电机2速度PID
	PID_SingleCalc(&motor2.pid.inner, motor2.targetSpeed, motor2.speed);
	if(motor2.pid.inner.output > 0)        //对应正转
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint32_t)motor2.pid.inner.output);
		IN3(0);
		IN4(1);
	}
	else				  //对应反转					
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint32_t)(-motor2.pid.inner.output));
		IN3(1);
		IN4(0);
	}
}
uint8_t Motor_Send(void)
{
	static uint32_t output1 = 0;
	static uint32_t output2 = 0;
	static uint8_t index = 1;
    PID_AngleCalc(&motor1.pid, motor1.targetAngle, motor1.totalAngle, motor1.speed);
	PID_AngleCalc(&motor2.pid, motor2.targetAngle, motor2.totalAngle, motor2.speed);
	output1 = motor1.pid.output;
	output2 = motor2.pid.output;
	if(output1 > 0)        //对应正转
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)motor1.pid.output);
		IN1(1);
		IN2(0);
	}
	else				  //对应反转					
	{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint32_t)(-motor1.pid.output));
		IN1(0);
		IN2(1);
	}

	if(output2 > 0)        //对应正转
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
	if(abs(output1) - 0 < 1 && abs(output2) - 0 < 1){
		index = 0;
		return index;
	}
	return index;
}

uint8_t Line_Control(uint32_t x, uint32_t y){
	uint32_t dx = (x-line.x)/POINT_SPACING;
	uint32_t dy = (y-line.y)/POINT_SPACING;
	for(uint8_t t=0; t<POINT_SPACING + 1; t++){
		line.line_dl = sqrt(pow((15+line.x+dx), 2.0) + pow((115-line.y-dy),2.0))-line.line_left;
		line.line_dr = sqrt(pow((95+line.x+dx), 2.0) + pow((115-line.y-dy),2.0))-line.line_right;
		line.line_left += line.line_dl;
		line.line_right += line.line_dr;
		motor1.targetAngle = motor1.totalAngle + line.line_dl/METER_CYCLE*(4*13*RR);
		motor2.targetAngle = motor2.totalAngle + line.line_dr/METER_CYCLE*(4*13*RR);
		x = HAL_GetTick();
		while(Motor_Send()){//分别到达每一个点
			y = HAL_GetTick();
			while(y-x <= 20){
				y = HAL_GetTick();
			}
			x = HAL_GetTick();	
		}
	}
	printf("get point!\r\n");
	return 1;
}
uint8_t cycle_Control(void){
	uint32_t x = line.x;
	uint32_t y = line.y;
	uint32_t dx = 0.01;
	uint32_t dy = 0;
	for(uint16_t i=0; i<50; i++){
		dy = sqrt(pow(0.25,2.0)-pow(0.25 - (i+1)*dx,2.0));
		Line_Control(x+(i+1)*dx, y+dy);
	}
	for(uint16_t i=50; i>0; i--){
		dy = -sqrt(pow(0.25,2.0)-pow(0.25 - (i+1)*dx,2.0));
		Line_Control(x+(i+1)*dx, y+dy);
	}
	return 1;
}

