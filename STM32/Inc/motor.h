#ifndef __motor_H
#define __motor_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

extern osThreadId motorControlThreadHandle;

typedef struct motorPid_t{
	float setPoint;
	float out;
	float error;
	float lastError;
	float P;
	float I;
	float D;
	float kP;
	float kI;
	float kD;
	float maxI;
	float maxOut;
}__attribute__((packed)) motorPid_t;

typedef struct motor_t{
  GPIO_TypeDef* dirPort;
	uint32_t dirPin;
	TIM_HandleTypeDef *encTim;
	TIM_HandleTypeDef *pwmTim;
	uint32_t pwmCh;
	int32_t encCountPrev;
	int32_t encCountRaw;
	int32_t encCountRound;
	int64_t encCountTotal;
	float encMeterToCount;
}__attribute__((packed)) motor_t;

void motorControlThreadFunction(void const * argument);

#ifdef __cplusplus
}
#endif
#endif /*__ motor_H */