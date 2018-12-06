#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "task.h"
#include "motor.h"
#include <string.h>

#define ENCODER_MAX_COUNT			65535		//basically size of timer register
#define GEAR_REDUCTION				298			//n20 motor with 1:298 gear reduction
#define ROLLER_DIAMETER				0.006		//meters, aprox number
#define ENCODER_CNT_PER_REV		7 			//on the motor spindle, one rev gives 7 encoder pulses

osThreadId motorControlThreadHandle;
extern float motorPosCmd[5];

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim19;

motor_t motors[5];
motorPid_t pids[5];

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX) {
	  *a = ABS_MAX;
	  return 1;
  }

  if (*a < -ABS_MAX){
	  *a = -ABS_MAX;
	  return 1;
  }

  return 0;

}

void motorInit(motor_t* motor) {

	HAL_TIM_Encoder_Start(motor->encTim, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(motor->pwmTim, motor->pwmCh);

}

void motorSetPwm(motor_t* motor, float pwmSet) {

	abs_limit(&pwmSet, ENCODER_MAX_COUNT);

	if (pwmSet > 0) {
		HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, 0);
		__HAL_TIM_SET_COMPARE(motor->pwmTim, motor->pwmCh, (uint16_t)pwmSet);
	} else {
		HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, 1);
		__HAL_TIM_SET_COMPARE(motor->pwmTim, motor->pwmCh, (uint16_t)-pwmSet);
	}

}

void motorControlThreadFunction(void const* argument) {

	motors[0].dirPort = DIR5_GPIO_Port;		//mapping the motors from mechanical order to electrical connections
	motors[0].dirPin = DIR5_Pin;
	motors[0].encTim = &htim19;
	motors[0].pwmTim = &htim12;
	motors[0].pwmCh = TIM_CHANNEL_2;

	motors[1].dirPort = DIR3_GPIO_Port;
	motors[1].dirPin = DIR3_Pin;
	motors[1].encTim = &htim4;
	motors[1].pwmTim = &htim15;
	motors[1].pwmCh = TIM_CHANNEL_2;

	motors[2].dirPort = DIR1_GPIO_Port;
	motors[2].dirPin = DIR1_Pin;
	motors[2].encTim = &htim2;
	motors[2].pwmTim = &htim16;
	motors[2].pwmCh = TIM_CHANNEL_1;

	motors[3].dirPort = DIR2_GPIO_Port;
	motors[3].dirPin = DIR2_Pin;
	motors[3].encTim = &htim3;
	motors[3].pwmTim = &htim17;
	motors[3].pwmCh = TIM_CHANNEL_1;

	motors[4].dirPort = DIR4_GPIO_Port;
	motors[4].dirPin = DIR4_Pin;
	motors[4].encTim = &htim5;
	motors[4].pwmTim = &htim12;
	motors[4].pwmCh = TIM_CHANNEL_1;

	for(uint8_t i = 0; i < 5; i++) {

		motorInit(&motors[i]);
		pids[i].kP = 2500;			//current PID profile is stable for 6-12v in
		pids[i].kI = 50;
		pids[i].kD = 10000;
		pids[i].maxI = 1000000;
		pids[i].maxOut = 65535;
		motors[i].encMeterToCount = ENCODER_CNT_PER_REV * GEAR_REDUCTION / (3.14159 * ROLLER_DIAMETER);

	}

  while(1) {

		for(uint8_t i = 0; i < 5; i++) {

			//update encoder data
			motors[i].encCountRaw = __HAL_TIM_GET_COUNTER(motors[i].encTim);
			if 			(motors[i].encCountRaw - motors[i].encCountPrev > ENCODER_MAX_COUNT / 2) motors[i].encCountRound--;
			else if (motors[i].encCountRaw - motors[i].encCountPrev < -ENCODER_MAX_COUNT / 2) motors[i].encCountRound++;
			motors[i].encCountTotal = motors[i].encCountRaw + motors[i].encCountRound * ENCODER_MAX_COUNT;
			motors[i].encCountPrev = motors[i].encCountRaw;

			//calc PID
			pids[i].setPoint = motorPosCmd[i] * motors[i].encMeterToCount;
			uint8_t maxed = 0;
			pids[i].error = pids[i].setPoint + motors[i].encCountTotal;
			pids[i].P = pids[i].kP * pids[i].error;
			pids[i].D = pids[i].kD * (pids[i].error - pids[i].lastError);
			pids[i].out = pids[i].P + pids[i].D + pids[i].I * pids[i].kI;
			maxed = abs_limit(&pids[i].out, pids[i].maxOut);
			pids[i].I = maxed ? 0 : pids[i].error + pids[i].I;
			abs_limit(&pids[i].I, pids[i].maxI);
			pids[i].lastError = pids[i].error;

			motorSetPwm(&motors[i], pids[i].out);

		}

		osDelay(10);

  }

}
