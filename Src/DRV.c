#include "DRV.h"
#include "cmsis_os.h"
#include "main.h"

uint16_t OLD_TIM_DRV_A = 0;
uint32_t OLD_PWM_DRV_A = 1500; //1057..1947
uint16_t OLD_TIM_DRV_B = 0;
uint32_t OLD_PWM_DRV_B = 1500; //1057..1947
uint16_t OLD_TIM_DRV_C = 0;
uint32_t OLD_PWM_DRV_C = 1500; //1057..1947
uint16_t OLD_TIM_DRV_D = 0;
uint32_t OLD_PWM_DRV_D = 1500; //1057..1947

GPIO_PinState direction_up = GPIO_PIN_SET;
GPIO_PinState direction_down = GPIO_PIN_RESET;

extern SPI_HandleTypeDef hspi3;
//extern UART_HandleTypeDef huart7;

uint8_t DRV_WriteRegister(uint8_t num_channel, uint8_t addr, uint16_t data)
{
	uint8_t data_mas[3];
	uint8_t pdata_spi[3];
	uint8_t data_2 = (uint8_t)data;
	uint8_t data_1 = (uint8_t)(data>>8);
	
	data_mas[0] = data_2;
	data_mas[1] = data_1;
	data_mas[2] = addr;
	
	//char buf[100];
	
	switch(num_channel)
	{
		case 1: //A
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_SPI_Transmit(&hspi3, data_mas, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		
			pdata_spi[0] = 0x00;
			pdata_spi[1] = 0x00;
			pdata_spi[2] = 0x80 + addr;
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_SPI_Receive(&hspi3, pdata_spi, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			//HAL_UART_Transmit(&huart3, pdata_spi, 3, HAL_MAX_DELAY);
			break;
		case 2: //B
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_SPI_Transmit(&hspi3, data_mas, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			break;
		case 3: //C
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_SPI_Transmit(&hspi3, data_mas, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			break;
		case 4: //D
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_SPI_Transmit(&hspi3, data_mas, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
			break;
	}
	
	//sprintf(buf, "Test connection %X %X %X %X\n\r", pdata_spi[0], pdata_spi[1], data_mas[0], data_mas[1]);
	//HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
	
	
	if((data_mas[0] == pdata_spi[0]) && (data_mas[1] == pdata_spi[1]))
		return 0;
	return 1;
}

uint16_t DRV_SetSpeed(uint32_t NEW_PWM, uint16_t OLD_PWM, uint16_t MAX_ACCEL, uint16_t MAX_DECEL, uint16_t MIN_PWM, uint16_t MAX_PWM)
{
	uint32_t OUTPUT_PWM = 0;
	if((NEW_PWM >= MIN_PWM) && (NEW_PWM <= MAX_PWM)){
		if(OLD_PWM < NEW_PWM){ 
			//acceleration
			if((NEW_PWM - OLD_PWM) > MAX_ACCEL)
				OUTPUT_PWM = OLD_PWM + MAX_ACCEL;
			else
				OUTPUT_PWM = NEW_PWM;
		}else{ 
			//deceleration
			if((OLD_PWM - NEW_PWM) > MAX_DECEL)
				OUTPUT_PWM = OLD_PWM - MAX_DECEL;
			else
				OUTPUT_PWM = NEW_PWM;
		}
	}
	if((OUTPUT_PWM > MIN_PWM) && (OUTPUT_PWM < MAX_PWM))
		return OUTPUT_PWM;
	else
		return OLD_PWM;
}

double DRV_ConvertPWMtoCLKOUT(double PWM, uint8_t car_mode)
{
	double y;
	y = 0.194118*PWM - 289.235; //100% X8R
	//y = 0.222222*PWM - 333.333; //100%
	//y = 0.098039*x - 146.078; //50%
	if(y > 99.0)
		y = 99.0;
	if(y < -99.0)
		y = -99.0;
	return y;
}

void DRV_Init(uint8_t num_channel)
{	
	uint8_t errors_cnt = 0;
	
	switch(num_channel)
	{
		case 1: //A
			errors_cnt += DRV_WriteRegister(num_channel, 0, DRV_REGISTR_0_Up); //9151
		break;
		case 2: //B
			errors_cnt += DRV_WriteRegister(num_channel, 0, 0x9111); //9151
		break;
		case 3: //C
			errors_cnt += DRV_WriteRegister(num_channel, 0, 0x9511); //9151
		break;
		case 4: //D
			errors_cnt += DRV_WriteRegister(num_channel, 0, 0x9111); //9151
		break;
	}
	errors_cnt += DRV_WriteRegister(num_channel, 1, 		DRV_REGISTR_1);
	errors_cnt += DRV_WriteRegister(num_channel, 2, 		DRV_REGISTR_2);
	errors_cnt += DRV_WriteRegister(num_channel, 3, 		DRV_REGISTR_3);
	errors_cnt += DRV_WriteRegister(num_channel, 4, 		DRV_REGISTR_4);
	errors_cnt += DRV_WriteRegister(num_channel, 5, 		DRV_REGISTR_5);
	errors_cnt += DRV_WriteRegister(num_channel, 6, 		DRV_REGISTR_6);
	errors_cnt += DRV_WriteRegister(num_channel, 7, 		DRV_REGISTR_7);
	errors_cnt += DRV_WriteRegister(num_channel, 8, 		DRV_REGISTR_8);
	errors_cnt += DRV_WriteRegister(num_channel, 9, 		DRV_REGISTR_9);
	errors_cnt += DRV_WriteRegister(num_channel, 10, 		DRV_REGISTR_10);
	errors_cnt += DRV_WriteRegister(num_channel, 11, 		DRV_REGISTR_11);
	DRV_WriteRegister(num_channel, 0x2A, 	DRV_REGISTR_2A);
	
	#ifdef DEBBUG
	if(errors_cnt)
	{
		sprintf(buf, "DRV %d is not initialization. We have %d errors.\n\r", num_channel, errors_cnt);
		HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
	}
	else
	{
		sprintf(buf, "DRV %d initialization.\n\r", num_channel);
		HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
	}
	#endif
}

void DRV_SetPWM(uint8_t NUM_DRV, uint32_t INPUT_PWM)
{
	uint16_t DELTA_TIM_DRV = 0;
	double y = 0;
	
	//Check limit input PWM
	if((INPUT_PWM < MIN_INPUT_PWM_DRV) || (INPUT_PWM > MAX_INPUT_PWM_DRV))
		return;
	
	switch(NUM_DRV)
	{
		case 1: //A
			
			if(TIM12->CNT >= OLD_TIM_DRV_A)
				DELTA_TIM_DRV = TIM12->CNT - OLD_TIM_DRV_A;
			else
				DELTA_TIM_DRV = TIM12->CNT + (0xFFFF - OLD_TIM_DRV_A);
				
			if(DELTA_TIM_DRV > DELTA_TIME_PWM_DRV)
			{
				if(OLD_PWM_DRV_A > 1500)
					OLD_PWM_DRV_A = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRV_A, MAX_ACCELERATION_INPUT_PWM_DRV, MAX_DECELERATION_INPUT_PWM_DRV, MIN_INPUT_PWM_DRV, MAX_INPUT_PWM_DRV);
				else
					OLD_PWM_DRV_A = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRV_A, MAX_DECELERATION_INPUT_PWM_DRV, MAX_ACCELERATION_INPUT_PWM_DRV, MIN_INPUT_PWM_DRV, MAX_INPUT_PWM_DRV);
				OLD_TIM_DRV_A = TIM12->CNT;
			}
			
			if((OLD_PWM_DRV_A < 1490) && (OLD_PWM_DRV_A > 1440))
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //Channel A
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //Channel A
			}
	
			y = DRV_ConvertPWMtoCLKOUT((double)OLD_PWM_DRV_A, 0);
			
			if(y < 0)
			{
				y *= -1;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , direction_up); //Channel A
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , direction_down); //Channel A
			}
			
			TIM1->CCR1 = (uint32_t)y;
		break;/*
		case 2: //B
			
			if(TIM12->CNT >= OLD_TIM_DRW_B)
				DELTA_TIM_DRW = TIM12->CNT - OLD_TIM_DRW_B;
			else
				DELTA_TIM_DRW = TIM12->CNT + (0xFFFF - OLD_TIM_DRW_B);
				
			if(DELTA_TIM_DRW > DELTA_TIME_PWM_DRW){
				if(OLD_PWM_DRW_B > 1500)
					OLD_PWM_DRW_B = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_B, MAX_ACCELERATION_INPUT_PWM_DRW, MAX_DECELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				else
					OLD_PWM_DRW_B = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_B, MAX_DECELERATION_INPUT_PWM_DRW, MAX_ACCELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				OLD_TIM_DRW_B = TIM12->CNT;
			}
			
			y = DRV_GetPWM((double)OLD_PWM_DRW_B, 0);
			
			if(y < 0){
				y *= -1;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Channel B
			}else{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Channel B
			}

			TIM4->CCR2 = (uint32_t)y;
		break;
		case 3: //C
			
			if(TIM12->CNT >= OLD_TIM_DRW_C)
				DELTA_TIM_DRW = TIM12->CNT - OLD_TIM_DRW_C;
			else
				DELTA_TIM_DRW = TIM12->CNT + (0xFFFF - OLD_TIM_DRW_C);
				
			if(DELTA_TIM_DRW > DELTA_TIME_PWM_DRW){
				if(OLD_PWM_DRW_C > 1500)
					OLD_PWM_DRW_C = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_C, MAX_ACCELERATION_INPUT_PWM_DRW, MAX_DECELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				else
					OLD_PWM_DRW_C = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_C, MAX_DECELERATION_INPUT_PWM_DRW, MAX_ACCELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				OLD_TIM_DRW_C = TIM12->CNT;
			}
			
			y = DRV_GetPWM((double)OLD_PWM_DRW_C, 0);
			
			if(y < 0){
				y *= -1;
				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET); //Channel C
			}else{
				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET); //Channel C
			}

			TIM5->CCR3 = (uint32_t)y;
		break;
		case 4: //D
			
			if(TIM12->CNT >= OLD_TIM_DRW_D)
				DELTA_TIM_DRW = TIM12->CNT - OLD_TIM_DRW_D;
			else
				DELTA_TIM_DRW = TIM12->CNT + (0xFFFF - OLD_TIM_DRW_D);
				
			if(DELTA_TIM_DRW > DELTA_TIME_PWM_DRW){
				if(OLD_PWM_DRW_D > 1500)
					OLD_PWM_DRW_D = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_D, MAX_ACCELERATION_INPUT_PWM_DRW, MAX_DECELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				else
					OLD_PWM_DRW_D = DRV_SetSpeed(INPUT_PWM, OLD_PWM_DRW_D, MAX_DECELERATION_INPUT_PWM_DRW, MAX_ACCELERATION_INPUT_PWM_DRW, MIN_INPUT_PWM_DRW, MAX_INPUT_PWM_DRW);
				OLD_TIM_DRW_D = TIM12->CNT;
			}
			
			y = DRV_GetPWM((double)OLD_PWM_DRW_D, 0);
			
			if(y < 0){
				y *= -1;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); //Channel D
			}else{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); //Channel D
			}

			TIM2->CCR3 = (uint32_t)y;
		break;*/
	}
}
