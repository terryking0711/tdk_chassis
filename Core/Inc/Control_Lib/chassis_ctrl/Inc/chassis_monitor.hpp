/*
 * chassis_monitor.hpp
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */

#ifndef INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_MONITOR_HPP_
#define INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_MONITOR_HPP_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
void chassis_monitor(void);
void chassis_set_speed(float vx,float vy,float vz);
void chassis_give_speed();
void chassis_give_vel_only();
extern float vel_x, vel_y, vel_z;
extern float odom_x_m, odom_y_m, odom_yaw;   // 新增

#ifdef __cplusplus
}
#endif


#endif /* INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_MONITOR_HPP_ */
