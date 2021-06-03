

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_


typedef struct
{
	uint16_t position[2];
	int sendJoy[2];

}joystick;

void joystick_Start(ADC_HandleTypeDef *hadc, joystick *obiekt)
{
	HAL_ADC_Start_DMA(hadc, obiekt->position , 2);
}

void joystick_Stop(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Stop_DMA(hadc);
}

int voltageToAngleX(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[0]*1.514);;



		  return value;

}
int voltageToAngleY(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[1]*1.514);



		  return value;

}

void joystick_Control(joystick *obiekt)
{


	obiekt->sendJoy[0] = voltageToAngleX(obiekt);
	obiekt->sendJoy[1] = voltageToAngleY(obiekt);
}

#endif /* INC_JOYSTICK_H_ */
