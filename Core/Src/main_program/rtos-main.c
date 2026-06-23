/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
//#include "motor_ctrl.hpp"
#include "timers.h"
#include "motor_monitor.hpp"
#include "trace.hpp"
#include "uros_init.h"
#include "motor_config.h"
#include "chassis_monitor.hpp"
#include "Pinpoint_monitor.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern I2C_HandleTypeDef hi2c1;

uint16_t adcRead[7] = {0};

double LastCNT = 0;
double CNT = 500;
int turn =0;
double currentsp = 0;
int sec = 0,tct = 0;
//PinpointI2C pinpoint(&hi2c1);
//PinpointI2C::BulkData bd;

//TimerHandle_t xTimer;

//void motorTimerCallback(TimerHandle_t xTimer);

void StartDefaultTask(void *argument)
{
    uros_init();

    for (;;)
    {
        uros_agent_status_check();
        osDelay(10);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

//void motorTimerCallback(TimerHandle_t xTimer)
//{
//
//	chassis_monitor();
//	pinpoint_monitor();
//	sec++;
//}
//TODO:motor_PID,chassis,odometry,
//TODO: check other PWM output


