#include "stm32h7xx_hal.h"

#define DEBBUG 1

#define DRV_REGISTR_0_Up		0x9511
#define DRV_REGISTR_0_Down	0x9111
#define DRV_REGISTR_1 	0x40		//0x21
#define DRV_REGISTR_2 	0x364		//0x3B4
#define DRV_REGISTR_3 	0x6F82	//0xE800	//0x6F82	//0xE800
#define DRV_REGISTR_4 	0x247		//0x347		//0x247//0x240
#define DRV_REGISTR_5 	0x3007
#define DRV_REGISTR_6 	0x84B1
#define DRV_REGISTR_7 	0x3BA
#define DRV_REGISTR_8 	0x12C
#define DRV_REGISTR_9 	0x250
#define DRV_REGISTR_10 	0xF064
#define DRV_REGISTR_11 	0x7D0
#define DRV_REGISTR_2A 	0

#define SERVO_1_PWM_ZERO 1500

#define I2C_ADDRESS_DRV 0x80
#define I2C_ADDRESS_SERVO 0x81

#define INA_NUM 			0x84
#define INA_PSU_3V3 	0x80
#define INA_PSU_5V0 	0x82
#define INA_PSU_8V4A 	0x84
#define INA_PSU_8V4B 	0x86
#define INA_PSU_5VB 	0x88
#define INA_PSU_12V 	0x8A
#define INA_INPUT 		0x8E

uint16_t MedianFilter(uint16_t *data, uint16_t new_data);
uint8_t DRV_WriteRegister(uint8_t num_channel, uint8_t addr, uint16_t data);
uint16_t DRV_SetSpeed(uint32_t NEW_PWM, uint16_t OLD_PWM, uint16_t MAX_ACCEL, uint16_t MAX_DECEL, uint16_t MIN_PWM, uint16_t MAX_PWM);
double DRV_ConvertPWMtoCLKOUT(double PWM, uint8_t car_mode);
void DRV_Init(uint8_t num_channel);
void DRV_SetPWM(uint8_t NUM_DRW, uint32_t INPUT_PWM);
void UART_SendMessage(const char *massage_uart);
void SERVO_SetPWM(uint8_t NUM_SERVO, uint16_t INPUT_PWM);
void SERVO_SetZero(uint8_t NUM_SERVO);
uint16_t GetEncoderAngle(uint8_t num_encoder);
int GetEncoderSpeed(uint16_t old_angle_encoder, uint16_t new_angle_encoder);
int GetMiddle_5(int new_variable);
uint16_t GetCount(void);
uint16_t GoTargetPoint(uint8_t num_encoder, int speed_0, uint16_t counter_0);
void CallibrateEncoderServo(uint8_t num_encoder);
void EncoderSetZero(uint8_t num_encoder);
void DRV_ChangeDir(void);
void INA_Init(uint8_t Num_INA, uint16_t Config_Param_INA, uint16_t Config_Shunt_INA);
void SendUint(uint16_t var, uint8_t command_type);
void SendFloat(float var, uint8_t command_type);
void SendInt(int var, uint8_t command_type);
