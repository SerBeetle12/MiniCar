#include "cmsis_os.h"
#include "system_func.h"
#include "main.h"
#include "string.h"
#include "task_os.h"

#include "DRV.h"

extern UART_HandleTypeDef huart7;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

char buf[100];
char old_message_uart[100];



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

