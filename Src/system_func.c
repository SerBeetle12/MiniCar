#include "cmsis_os.h"
#include "system_func.h"
#include "main.h"
#include "string.h"
#include "task_os.h"

#define MIN_INPUT_PWM_DRV 			900
#define MAX_INPUT_PWM_DRV 			2150
#define MAX_ACCELERATION_INPUT_PWM_DRV 15
#define MAX_DECELERATION_INPUT_PWM_DRV 15
#define DELTA_TIME_PWM_DRV 80

#define MIN_PWM_DRV_A 5
#define MAX_PWM_DRV_A 99
#define MIN_PWM_DRV_B 5
#define MAX_PWM_DRV_B 99
#define MIN_PWM_DRV_C 5
#define MAX_PWM_DRV_C 99
#define MIN_PWM_DRV_D 5
#define MAX_PWM_DRV_D 99
#define MAX_SPEED_DRV 1
#define MIDDLE_PWM_DRV 1500

extern UART_HandleTypeDef huart7;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

char buf[100];
char old_message_uart[100];

uint16_t OLD_TIM_DRV_A = 0;
uint32_t OLD_PWM_DRV_A = 1500; //1057..1947
uint16_t OLD_TIM_DRV_B = 0;
uint32_t OLD_PWM_DRV_B = 1500; //1057..1947
uint16_t OLD_TIM_DRV_C = 0;
uint32_t OLD_PWM_DRV_C = 1500; //1057..1947
uint16_t OLD_TIM_DRV_D = 0;
uint32_t OLD_PWM_DRV_D = 1500; //1057..1947

//Encoders
uint8_t addr_encoder = 0x80;
uint8_t data_i2c[3];
uint8_t high_byte = 0;
uint8_t low_byte = 0;
uint16_t angle_X = 0;
float Angle_Deg = 0;
I2C_HandleTypeDef name_i2c;
float A_encoder_5 = 0, B_encoder_5 = 0;
uint16_t NEW_POSITION_SERVO = 0;
uint16_t counter_wheel_1 = 32000;
uint16_t delta_angle_old = 0;
int encoder_speed_middle[5] = {0,0,0,0,0};
uint8_t encoder_speed_middle_cnt = 0;

//Encoder speed 
uint16_t delta_time = 1;
uint16_t encoder_timer_old = 0;
float result_speed;
uint16_t delta_angle = 0;
uint16_t time = 0;

//Regulator
extern uint16_t AP_log_counter;
extern uint16_t AP_log_ch1[MAX_LENGTH_AP_LOG];
extern int encoder_speed_1;

uint16_t target_PWM_ch1 = INPUT_PWM_DEFAULT;
float delta_speed = 0;
float delta_counter = 0;

int AP_log_ch1_speed[MAX_LENGTH_AP_LOG];
uint16_t AP_log_ch1_counter[MAX_LENGTH_AP_LOG];
float Regul_ch1_A = 0.0437f;//0.043146f;
float Regul_ch1_B = 1491.7566f;//1492.854f;
float Regul_ch1_KP = 1.0f;
float Regul_ch1_KI = 0.0f;

//Terminal
uint8_t uart_data[7];
union variables
{
	uint8_t u8[4];
	uint32_t u32;
	int i;
	float f;
} var_terminal;

GPIO_PinState direction_up = GPIO_PIN_SET;
GPIO_PinState direction_down = GPIO_PIN_RESET;


uint16_t MedianFilter(uint16_t *data, uint16_t new_data)
{
	uint16_t middle;
	data[2] = data[1];
	data[1] = data[0];
	data[0] = new_data;
	
	if ((data[0] <= data[1]) && (data[0] <= data[2])){
		middle = (data[1] <= data[2]) ? data[1] : data[2];
	}
	else
	{
		if ((data[1] <= data[0]) && (data[1] <= data[2])){
			middle = (data[0] <= data[2]) ? data[0] : data[2];
		}
		else
		{
			middle = (data[0] <= data[1]) ? data[0] : data[1];
		}
	}

	return middle;
}

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

void UART_SendMessage(const char *massage_uart)
{
	if(strcmp(old_message_uart, massage_uart))
	{
		HAL_UART_Transmit(&huart7, (uint8_t*)massage_uart, strlen(massage_uart), 100);
		strcpy(old_message_uart, massage_uart);
	}
}

void SERVO_SetPWM(uint8_t NUM_SERVO, uint16_t INPUT_PWM)
{
	switch(NUM_SERVO)
	{
		case 1:
			TIM15->CCR1 = INPUT_PWM;
		break;
		case 2:
			TIM15->CCR2 = INPUT_PWM;
		break;
	}
}


void SERVO_SetZero(uint8_t NUM_SERVO)
{
	switch(NUM_SERVO)
	{
		case 1:
			TIM15->CCR1 = SERVO_1_PWM_ZERO;
		break;
		case 2:
			TIM15->CCR2 = SERVO_1_PWM_ZERO;
		break;
	}
}

uint16_t GetEncoderAngle(uint8_t num_encoder)
{
	switch(num_encoder)
	{
		case 1:
			name_i2c = hi2c1;
			addr_encoder = 0x80;
		break;
		case 5:
			name_i2c = hi2c1;
			addr_encoder = 0x80;
		break;
	}
	
	data_i2c[0] = 0xFE;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	high_byte = data_i2c[0];
	data_i2c[0] = 0xFF;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	low_byte = data_i2c[0];
	
	angle_X = (uint16_t)low_byte<<6 | high_byte;
	Angle_Deg = 360.0f*((double)angle_X)/16383.0f;
	angle_X = (uint16_t)(Angle_Deg*10.0f);
	
	return angle_X;
}

int GetEncoderSpeed(uint16_t old_angle_encoder, uint16_t new_angle_encoder)
{
	time = TIM6->CNT;
	if(time > encoder_timer_old)
	{
		delta_time = time - encoder_timer_old;
	}
	else
	{
		delta_time = 0xFFFF + time - encoder_timer_old;
	}
	encoder_timer_old = TIM6->CNT;
	
	if(new_angle_encoder > old_angle_encoder)
	{
		if((new_angle_encoder - old_angle_encoder) > 1250)
		{
			delta_angle = 3600 - new_angle_encoder + old_angle_encoder;
			counter_wheel_1++;
		}
		else
		{
			delta_angle = new_angle_encoder - old_angle_encoder;
		}
	}
	else
	{
		if((old_angle_encoder - new_angle_encoder) > 1250)
		{
			delta_angle = 3600 + new_angle_encoder - old_angle_encoder;
			counter_wheel_1--;
		}
		else
		{
			delta_angle = old_angle_encoder - new_angle_encoder;
		}
	}
	if(delta_angle < 30) //3 degree
	{
		return 0;
	}
	if(delta_angle_old > delta_angle)
	{
		if((delta_angle_old - delta_angle) > 200)
		{
			//result_speed = delta_angle_old;
			delta_angle = delta_angle_old;
		}
	}
	else
	{
		if((delta_angle - delta_angle_old) > 200)
		{
			//result_speed = delta_angle_old;
			delta_angle = delta_angle_old;
		}
	}
	
	delta_angle_old = delta_angle;
	
	result_speed = (float)delta_angle/(float)delta_time*4000.0f/6.0f; //RPM
	
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
		result_speed *= -1.0f;
	//if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)
	//result_speed = (float)delta_angle/((float)delta_time/40000.0f);
	//result_speed = delta_angle;
	return (int)result_speed;
}

int GetMiddle_5(int new_variable)
{
	encoder_speed_middle[encoder_speed_middle_cnt++] = new_variable;
	if(encoder_speed_middle_cnt == 5)
		encoder_speed_middle_cnt = 0;
	return (encoder_speed_middle[0] + encoder_speed_middle[1] + encoder_speed_middle[2] + encoder_speed_middle[3] + encoder_speed_middle[4])/5;
}

uint16_t GetCount(void)
{
	return counter_wheel_1;
}

uint16_t GoTargetPoint(uint8_t num_encoder, int speed_0, uint16_t counter_0)
{
	delta_speed 	= (float)speed_0 		- (float)encoder_speed_1;
	delta_counter = (float)counter_0 	- (float)counter_wheel_1;
	
	//target_PWM_ch1 = (uint16_t)(Regul_ch1_A * (speed_0 + Regul_ch1_KP*delta_speed + Regul_ch1_KI*delta_counter) + Regul_ch1_B);
	target_PWM_ch1 = (uint16_t)(Regul_ch1_A * (float)speed_0 + Regul_ch1_B);
	
	if((target_PWM_ch1 > 1000) && (target_PWM_ch1 < 2000))
		return target_PWM_ch1;
	if(target_PWM_ch1 < 1000)
		return 1000;
	if(target_PWM_ch1 > 2000)
		return 2000;
	return 1500;
	//AP_log_ch1[AP_log_counter] = target_PWM_ch1;
}

void CallibrateEncoderServo(uint8_t num_encoder) //5 or 6
{
	uint16_t fi_min = 0;
	uint16_t fi_max = 0;
	
	SERVO_SetPWM(1, 1200);
	UART_SendMessage("Turn left.\n\r");
	osDelay(1500);
	fi_min = GetEncoderAngle(5);
	
	UART_SendMessage("Turn right.\n\r");
	SERVO_SetPWM(1, 1800);
	osDelay(1500);
	fi_max = GetEncoderAngle(5);
	
	A_encoder_5 = 600.0f/((float)fi_max - (float)fi_min);
	B_encoder_5 = 1800.0f - A_encoder_5*(float)fi_max;
	
	NEW_POSITION_SERVO = (uint16_t)(A_encoder_5*1800.0f + B_encoder_5);
	SERVO_SetPWM(1, NEW_POSITION_SERVO);
	
	UART_SendMessage("Encoder calibration.\n\r");
	/*name_i2c = hi2c1;
	addr_encoder = 0x80;
	
	data_i2c[0] = 0x16;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	high_byte = data_i2c[0];
	
	data_i2c[0] = 0x17;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	low_byte = data_i2c[0];
	
	sprintf(buf, "HB = %X LB = %X\n\r", high_byte, low_byte);
	HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);*/
}

void EncoderSetZero(uint8_t num_encoder)
{
	switch(num_encoder)
	{
		case 1:
			name_i2c = hi2c1;
			addr_encoder = 0x80;
		break;
		case 5:
			name_i2c = hi2c1;
			addr_encoder = 0x80;
		break;
	}
	data_i2c[0] = 0x16;
	data_i2c[1] = 0;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 2, 1);
	data_i2c[0] = 0x17;
	data_i2c[1] = 0;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 2, 1);
	
	data_i2c[0] = 0xFE;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	high_byte = data_i2c[0];
	
	data_i2c[0] = 0xFF;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	low_byte = data_i2c[0];
	
	data_i2c[0] = 0x17;
	data_i2c[1] = high_byte;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 2, 1);
	data_i2c[0] = 0x16;
	data_i2c[1] = low_byte+0x80;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 2, 1);
	
	data_i2c[0] = 0xFB;
	HAL_I2C_Master_Transmit(&name_i2c, addr_encoder, data_i2c, 1, 1);
	HAL_I2C_Master_Receive(&name_i2c, addr_encoder+1, data_i2c, 1, 1);
	high_byte = data_i2c[0];
	
	sprintf(buf, "Encoder %d set to zero position. Diagnostic flag = %X\n\r", num_encoder, high_byte);
	HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
}

void DRV_ChangeDir(void)
{
	//Define:
	//direction_up 		= GPIO_PIN_SET;
	//direction_down 	= GPIO_PIN_RESET;
	if(encoder_speed_1 == 0)
	{
		if(direction_up == GPIO_PIN_SET)
		{
			direction_up 		= GPIO_PIN_RESET;
			direction_down 	= GPIO_PIN_SET;
		}
		else
		{
			direction_up 		= GPIO_PIN_SET;
			direction_down 	= GPIO_PIN_RESET;
		}
	}
}

void INA_Init(uint8_t Num_INA, uint16_t Config_Param_INA, uint16_t Config_Shunt_INA)
{
	uint8_t data_i2c[3];
	
	data_i2c[0] = 0x00;
	data_i2c[1] = (uint8_t)(Config_Param_INA>>8);//0x43;
	data_i2c[2] = (uint8_t)Config_Param_INA;//0x3F;
	HAL_I2C_Master_Transmit(&hi2c4, Num_INA, data_i2c, 3, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive( &hi2c4, Num_INA, data_i2c, 2, HAL_MAX_DELAY);
	
	data_i2c[0] = 0x05;
	data_i2c[1] = (uint8_t)(Config_Shunt_INA>>8);//0x50;
	data_i2c[2] = (uint8_t)Config_Shunt_INA;//0x00;
	HAL_I2C_Master_Transmit(&hi2c4, Num_INA, data_i2c, 3, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive( &hi2c4, Num_INA, data_i2c, 2, HAL_MAX_DELAY);
}

void SendUint(uint16_t var, uint8_t command_type)
{
	var_terminal.u32 = var;
	uart_data[0] = 0x16;
	uart_data[1] = command_type;
	uart_data[2] = var_terminal.u8[0];
	uart_data[3] = var_terminal.u8[1];
	uart_data[4] = var_terminal.u8[2];
	uart_data[5] = var_terminal.u8[3];
	uart_data[6] = 0xFA;
	
	HAL_UART_Transmit(&huart7, uart_data, 7, 20);
}

void SendFloat(float var, uint8_t command_type)
{
	var_terminal.f = var;
	uart_data[0] = 0x0F;
	uart_data[1] = command_type;
	uart_data[2] = var_terminal.u8[0];
	uart_data[3] = var_terminal.u8[1];
	uart_data[4] = var_terminal.u8[2];
	uart_data[5] = var_terminal.u8[3];
	uart_data[6] = 0xFA;
	
	HAL_UART_Transmit(&huart7, uart_data, 7, 20);
}

void SendInt(int var, uint8_t command_type)
{
	var_terminal.i = var;
	uart_data[0] = 0xA0;
	uart_data[1] = command_type;
	uart_data[2] = var_terminal.u8[0];
	uart_data[3] = var_terminal.u8[1];
	uart_data[4] = var_terminal.u8[2];
	uart_data[5] = var_terminal.u8[3];
	uart_data[6] = 0xFA;
	
	HAL_UART_Transmit(&huart7, uart_data, 7, 20);
}

