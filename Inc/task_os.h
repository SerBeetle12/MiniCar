#include "stm32h7xx_hal.h"

#define TIME_SAFETYSWITCH 4000
#define INPUT_PWM_DEFAULT 1470
#define MAX_LENGTH_AP_LOG 600

void InitSystem(void);
void StartTask10(void const * argument);

