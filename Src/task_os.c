#include "cmsis_os.h"
#include "task_os.h"
#include "main.h"
#include "string.h"
#include "system_func.h"

enum STATE_CAR{
	SAFETY,
	ARM,
	DISARM,
	AUTOPILOT,
	CALIBRATION
} state_car;

enum STATE_BUTTON{
	PUSH,
	PREPUSH,
	PREUNPUSH,
	UNPUSH
} button;

uint32_t LED_time = 300;

//Input PWM
uint16_t data_ch1[3] = {INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT};
uint16_t data_ch2[3] = {INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT};
uint16_t data_ch3[3] = {INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT};
uint16_t data_ch4[3] = {INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT, INPUT_PWM_DEFAULT};
uint16_t PWM_ch1 = INPUT_PWM_DEFAULT;
uint16_t PWM_ch2 = INPUT_PWM_DEFAULT;
uint16_t PWM_ch3 = INPUT_PWM_DEFAULT;
uint16_t PWM_ch4 = INPUT_PWM_DEFAULT;

//Logging
uint16_t AP_log_counter = 0; //drv
uint16_t AP_log_ch1[MAX_LENGTH_AP_LOG]; //60 sec -> 1 min
uint16_t AP_log_ch2[MAX_LENGTH_AP_LOG]; //60 sec -> 1 min

//Encoder
extern uint16_t delta_time;
uint16_t encoder_angle_1 = 0;
uint16_t encoder_angle_1_old = 1800;
int encoder_speed_1 = 0;
uint16_t encoder_angle_5 = 0;

extern uint16_t counter_wheel_1;
extern int AP_log_ch1_speed[MAX_LENGTH_AP_LOG];
extern uint16_t AP_log_ch1_counter[MAX_LENGTH_AP_LOG];

//UART
uint8_t command_from_pc = 0xFF;
uint8_t recieve_data_uart[3];

extern UART_HandleTypeDef huart7;
extern SPI_HandleTypeDef hspi3;
extern osMessageQId QueueUARTHandle;
extern TIM_HandleTypeDef htim17;

void InitSystem(void)
{
	button = UNPUSH;
	state_car = DISARM;
}

//Wheel movement system task
void StartTask01(void const * argument)
{
  //float y;
	for(;;){
		if((state_car == ARM) || (state_car == AUTOPILOT))
			DRV_SetPWM(1, PWM_ch1);
		else
			DRV_SetPWM(1, INPUT_PWM_DEFAULT);
		/*y = 0.194118*PWM_ch1 - 289.235;
		if(y < 0){
			y *= -1;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , GPIO_PIN_SET); //Channel A
		}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , GPIO_PIN_RESET); //Channel A
		}
		TIM1->CCR1 = (uint32_t)y;*/
    osDelay(50);
  }
}
//Steering system task
void StartTask02(void const * argument)
{
	//QUEUE_t *msg;
  for(;;)
	{
		if((state_car == ARM) || (state_car == AUTOPILOT))
			SERVO_SetPWM(1, PWM_ch2);
		//else
		//	SERVO_SetZero(1);
		//strcpy(msg->Buf, "Message 2");
		//osMessagePut(QueueUARTHandle, (uint32_t)msg, 100);
		
    osDelay(50);
  }
}

//Wheel encoder task
void StartTask03(void const * argument)
{
	//QUEUE_t *msg;
	//char buf[100];
	
	
	EncoderSetZero(5);
	//CallibrateEncoderServo(5);
	//state_car = DISARM;
	//encoder_timer_old = TIM6->CNT;
  for(;;)
	{
		encoder_angle_1 = GetEncoderAngle(1);
		
		encoder_speed_1 = GetMiddle_5(GetEncoderSpeed(encoder_angle_1_old, encoder_angle_1));
		
		encoder_angle_1_old = encoder_angle_1;
		
		//sprintf(buf, "%d\t %d\t %d\n\r", encoder_angle_1, encoder_speed_1, delta_time);
		//sprintf(buf, "%d %d\n\r", encoder_angle_1, delta_time);
		//sprintf(buf, "$%d %d %d;", TIM3->CCR2, encoder_speed_1, GetCount());
		//sprintf(buf, "$%d;", GetCount());
		//HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 50);
		
    osDelay(3);
  }
}

//Wheel turn position monitor task
void StartTask04(void const * argument)
{
	
  for(;;)
	{
    osDelay(100);
  }
}

//Power supply system task
void StartTask05(void const * argument)
{
	
  for(;;)
	{
    osDelay(100);
  }
}

//Terminal
void StartTask06(void const * argument)
{
	//QUEUE_t *msg;
	//uint8_t addr_register = 0x80;
	#ifdef DEBBUG
	uint8_t pdata_spi[3];
	char buf[100];
	#endif
  for(;;)
  {
		SendUint(PWM_ch1, 0xB0);
    osDelay(10);
		SendUint(PWM_ch2, 0xB1);
    osDelay(10);
		SendUint(PWM_ch3, 0xB2);
    osDelay(10);
		SendUint(PWM_ch4, 0xB3);
    osDelay(10);
		SendFloat(2.1f, 0xB4);
    osDelay(10);
		SendInt(encoder_speed_1, 0xC2);
    osDelay(10);
		SendUint(counter_wheel_1, 0xC3);
    osDelay(10);
		
		
		HAL_UART_Receive_IT(&huart7, recieve_data_uart, 3);
		//else
		//	command_from_pc = recieve_data_uart[1];
		#ifdef DEBBUG
		DRV_WriteRegister(1, 0x2A, 	0);
		
		pdata_spi[0] = 0x00;
		pdata_spi[1] = 0x00;
		pdata_spi[2] = 0xAA;
		
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_SPI_Receive(&hspi3, pdata_spi, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		
		if(pdata_spi[0])
		{
			if(pdata_spi[0]&0x01)
			{
				strcpy(buf, "ERROR DRV: Motor overcurrent (OCP)\n\r");
				HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
			}
			if(pdata_spi[0]&0x02)
			{
				strcpy(buf, "ERROR DRV: Charge pump overcurrent (CPOC)\n\r");
				HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
			}
			if(pdata_spi[0]&0x04)
			{
				strcpy(buf, "ERROR DRV: Overtemperature (OTS)\n\r");
				HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
			}
			if(pdata_spi[0]&0x08)
			{
				strcpy(buf, "ERROR DRV: VM undervoltage (UVLO)\n\r");
				HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
			}
			if(pdata_spi[0]&0x10)
			{
				strcpy(buf, "ERROR DRV: Charge pump undervoltage (CPFAIL)\n\r");
				HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
			}
		}
		
		//sprintf(buf, "TELEM = %d %d %d %d\n\r", TIM3->CCR2, encoder_speed_1, GetCount(), delta_time);
		//sprintf(buf, "$%d %d;",encoder_speed_1, GetCount());
		//sprintf(buf, "%d %d %d %d\n", AP_log_ch1_speed[AP_log_counter], encoder_speed_1, AP_log_ch1_counter[AP_log_counter], GetCount());
		//sprintf(buf, "%d %d\n", encoder_speed_1, PWM_ch1);
		sprintf(buf, "%d %d\n", encoder_speed_1, GetCount());
		//sprintf(buf, "%i %d\n", encoder_speed_1, HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2));
		HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 50);
		//sprintf(buf, "PWM RC = %d\n\r", PWM_ch1);
		//HAL_UART_Transmit(&huart7, (uint8_t*)buf, strlen(buf), 100);
		#endif
		/*
		addr_register = 0x80;
		while(addr_register != 0x8C)
		{
			pdata_spi[0] = 0x00;
			pdata_spi[1] = 0x00;
			pdata_spi[2] = addr_register;
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_SPI_Receive(&hspi3, pdata_spi, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_UART_Transmit(&huart3, pdata_spi, 3, HAL_MAX_DELAY);
			
			pdata_spi[0] = addr_register;
			HAL_UART_Transmit(&huart3, pdata_spi, 1, HAL_MAX_DELAY);
			
			HAL_Delay(10);
			addr_register++;
		}*/
		/*osEvent evt = osMessageGet(QueueUARTHandle, 100);
		msg = (QUEUE_t*)evt.value.p;
		HAL_UART_Transmit(&huart3, (uint8_t*)msg->Buf, strlen(msg->Buf), 100);*/
		//strcpy(buf, "Test connection uart\n\r");
		//sprintf(buf, "State car = %d\n\r", state_car);
		//HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
		//HAL_UART_Transmit_DMA(&huart3, (uint8_t*)buf, sizeof(buf)-1);
		//sprintf(buf, "IN_PWM = %d %d %d %d\n\r", PWM_ch1, PWM_ch2, PWM_ch3, PWM_ch4);
		//HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
		
		osDelay(100);
  }
}

//Logging system task
void StartTask07(void const * argument)
{
  for(;;)
  {
	
		osDelay(10);
  }
}

//Remote control system task
void StartTask08(void const * argument)
{
  for(;;)
  {
		/*if(command_from_pc == 0xA1) //start record movement
		{
			UART_SendMessage("Start record movement.\n\r");
			if(AP_log_ch1_counter == MAX_LENGTH_AP_LOG)
			{
				AP_log_ch1_counter = 0;
				command_from_pc = 0xFF;
			}
			AP_log_ch1[AP_log_ch1_counter++] = MedianFilter(data_ch1, TIM3->CCR2);
		}
		if(command_from_pc == 0xA0) //start manual mode
		{
			UART_SendMessage("Start manual mode.\n\r");
			PWM_ch1 = MedianFilter(data_ch1, TIM3->CCR2);
			PWM_ch2 = MedianFilter(data_ch2, TIM3->CCR3 - TIM3->CCR2);
		}*/
		if(state_car == ARM)
		{
			//PWM_ch1 = MedianFilter(data_ch1, TIM3->CCR2);
			//PWM_ch2 = MedianFilter(data_ch2, TIM3->CCR3 - TIM3->CCR2);
			//PWM_ch3 = MedianFilter(data_ch3, TIM3->CCR4 - TIM3->CCR3);
			//PWM_ch4 = MedianFilter(data_ch4, TIM8->CCR1);
			PWM_ch1 = MedianFilter(data_ch1, TIM3->CCR3);
			PWM_ch2 = MedianFilter(data_ch2, TIM3->CCR2);
			PWM_ch3 = MedianFilter(data_ch3, TIM8->CCR1);
			PWM_ch4 = MedianFilter(data_ch4, TIM3->CCR4);
		}
		osDelay(10);
  }
}

//System state task
void StartTask09(void const * argument)
{
  for(;;)
  {
		switch(command_from_pc)
		{
			case 0xA0:
				state_car = ARM;
				command_from_pc = 0xFF;
				UART_SendMessage("Manual mode active.\n\r");
			break;
			case 0xA1: //start record movement
				HAL_TIM_Base_Start_IT(&htim17);
				UART_SendMessage("Start record movement.\n\r");
				if(AP_log_counter == MAX_LENGTH_AP_LOG)
				{
					AP_log_counter = 0;
					command_from_pc = 0xFF;
					UART_SendMessage("Autopilot time limit exceeded.\n\r");
				}
			break;
			case 0xA2: //start autopilot
				if(state_car != AUTOPILOT)
					DRV_ChangeDir();
				state_car = AUTOPILOT;
				UART_SendMessage("Start autopilot.\n\r");
				if(AP_log_counter == 0)
				{
					command_from_pc = 0xFF;
					PWM_ch1 = INPUT_PWM_DEFAULT;
					UART_SendMessage("END autopilot.\n\r");
					DRV_ChangeDir();
					HAL_TIM_Base_Stop(&htim17);
					state_car = ARM;
				}
			break;
			case 0xA3: //stop record movement
				state_car = AUTOPILOT;
				PWM_ch1 = GoTargetPoint(1, 3000, 32000);
				//command_from_pc = 0xFF;
			break;
			case 0xA4: //stop record movement
				state_car = AUTOPILOT;
				PWM_ch1 = GoTargetPoint(1, 5000, 32000);
				//command_from_pc = 0xFF;
			break;
			case 0xA5: //stop record movement
				DRV_ChangeDir();
				command_from_pc = 0xFF;
			break;
			default:
				
			break;
		}
		osDelay(100);
  }
}


//Button task
void StartTask10(void const * argument)
{
	//char buf[10];
  for(;;)
	{
		if(HAL_GPIO_ReadPin(Btn_SS_GPIO_Port, Btn_SS_Pin) == GPIO_PIN_SET)
		{
			if((button == UNPUSH) && (TIM16->CNT > TIME_SAFETYSWITCH))
			{
				TIM16->CNT = 0;
				button = PREPUSH;
			}
			if((button == PREPUSH) && (TIM16->CNT > TIME_SAFETYSWITCH))
			{
				LED_time = 100;
				button = PUSH;
				state_car = ARM;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //Channel A
				#ifdef DEBBUG
				HAL_UART_Transmit(&huart7, (uint8_t*)("ARM!\n\r"), 6, 100);
				#endif
				TIM16->CNT = 0;
			}
			if((button == PUSH) && (TIM16->CNT > TIME_SAFETYSWITCH))
			{
				TIM16->CNT = 0;
				button = PREUNPUSH;
			}
			if((button == PREUNPUSH) && (TIM16->CNT > TIME_SAFETYSWITCH))
			{
				LED_time = 300;
				button = UNPUSH;
				state_car = DISARM;
				#ifdef DEBBUG
				HAL_UART_Transmit(&huart7, (uint8_t*)("DISARM!\n\r"), 9, 100);
				#endif
				osDelay(3000);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //Channel A
				TIM16->CNT = 0;
			}
		}
		else
		{
			if(button == PREPUSH)
				button = UNPUSH;
			if(button == PREUNPUSH)
				button = PUSH;
		}
    HAL_GPIO_TogglePin(Led_SS_GPIO_Port, Led_SS_Pin);
    osDelay(LED_time);
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if((recieve_data_uart[0] == 0xF0) && (recieve_data_uart[2] == 0xAB))
		command_from_pc = recieve_data_uart[1];
	HAL_UART_Receive_IT(&huart7, recieve_data_uart, 3);
}

