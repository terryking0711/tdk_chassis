/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "timers.h"
#include "motor_monitor.hpp"
#include "trace.hpp"
#include "uros_init.h"
#include "motor_config.h"
#include "chassis_monitor.hpp"
#include "Pinpoint_monitor.hpp"
#include "config.h"

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
int turn = 0;
double currentsp = 0;
int sec = 0, tct = 0;

/*
 * 若未啟用 Pinpoint，提供本地 dummy pose。
 * 避免沒有 I2C 裝置時還去讀 Pinpoint。
 */

void StartDefaultTask(void *argument)
{
    /*
     * 1. micro-ROS transport first.
     */
	HAL_TIM_Base_Start_IT(&htim5);
    uros_init();

#if ENABLE_MOTOR_CONTROL
    /*
     * 2. Motor timer/PWM/encoder init.
     * 沒有接馬達時通常也可以執行。
     */
    motor_init();
#endif

#if ENABLE_PINPOINT
    /*
     * 3. 只有接上 Pinpoint 硬體後才啟用。
     */
    pinpoint_init();
#endif

    for (;;)
    {
        /*
         * 4. micro-ROS state machine.
         * 這裡會處理 agent waiting / connected / executor spin。
         */
        uros_agent_status_check();

#if ENABLE_CHASSIS_CONTROL
        /*
         * 5. cmd_vel -> chassis.
         * cmd_vel callback 會更新 vx, vy, vz。
         */
//        chassis_set_speed(0.3, 0.0, 0.0);
//        chassis_give_speed();
#endif

#if ENABLE_PINPOINT
        /*
         * 6A. 有 Pinpoint 時，用 Pinpoint pose。
         */
        pinpoint_monitor();        // 先刷新 bd（原本順序反了，會讀到舊資料）
        update_pinpoint_pose();    // 再把 bd 轉成 SI 寫進 odom_* / vel_*
        update_pose(odom_x_m, odom_y_m, odom_yaw, vel_x, vel_y, vel_z);
#else
        /*
         * 6B. 沒有 Pinpoint 時，用輪式運動學積分出的 odom。
         */
        update_pose(odom_x_m, odom_y_m, odom_yaw, vel_x, vel_y, vel_z);
#endif

        osDelay(10);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
		  {
			sec++;
			tct++;
		    // chassis_monitor();
			chassis_set_speed(vx, vy, vz);
#if ENABLE_PINPOINT
		    chassis_give_vel_only();   // 位置由 Pinpoint 主導，只更新速度
#else
		    chassis_give_speed();
#endif
		  }
    /*
     * 只保留 HAL tick。
     * 不要在 interrupt 裡跑 chassis / Pinpoint / micro-ROS。
     */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}
