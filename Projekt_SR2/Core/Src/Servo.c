#include "Servo.h"
#include "stm32f4xx.h"

int previous = 1600;
int previous1 = 7800;

void servo_Init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

}

void moveServo1(TIM_HandleTypeDef *htim,int angle)
{

	if(previous<angle)
	{

	for (; previous < angle; previous+=10) {
		if(previous>angle)
		{
			previous = angle;
		}
	  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, previous);
	 HAL_Delay(60);
	}
	}

	else
	{

		for (; previous > angle; previous -= 10) {
			if (previous < angle) {
				previous = angle;
			}
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, previous);
			HAL_Delay(60);
		}
	}
}

void moveServo2(TIM_HandleTypeDef *htim,int angle)
{
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, Theta2(90));

	if(previous1<angle)
	{
	for (; previous1 < angle; previous1+=15) {
		if(previous1>angle)
		{
			previous1 = angle;
		}
	  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, previous1);
	 HAL_Delay(40);
	}
	}

	else
	{
		for (; previous1 > angle; previous1 -= 15) {
			if (previous1 < angle) {
				previous1 = angle;
			}
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, previous1);
			HAL_Delay(40);
		}
	}
}
