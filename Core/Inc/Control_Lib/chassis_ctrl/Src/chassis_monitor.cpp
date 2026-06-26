/*
 * chassis_monitor.cpp
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */




#include "chassis.hpp"
#include "chassis_monitor.hpp"
#include "motor_monitor.hpp"
//#include "Pinpoint.hpp"

extern MotorController Motor_FR;
extern MotorController Motor_FL;
extern MotorController Motor_BR;
extern MotorController Motor_BL;
Chassis chassis(&Motor_FR, &Motor_FL , &Motor_BR, &Motor_BL);
float Vx_goal = 0.0;
float Vy_goal = 0.0;
float W_goal = 0.0;
float w_goal,x_goal,y_goal;
//extern PinpointI2C::BulkData bd;
float x_error,y_error;
float vel_x, vel_y, vel_z;
float odom_x_m, odom_y_m, odom_yaw;   // 新增：輪式 odom 的 SI 輸出(公尺/弧度)


void chassis_monitor(void) {
//	if(bd.pos_y_mm< y_goal){
    chassis.setSpeed(Vx_goal,Vy_goal, W_goal);
//	}else{
//		chassis.setSpeed(0,0,0);
//	}
	chassis.getLocation();
}

void chassis_set_speed(float vx,float vy,float vz)//阿包版的chassis monitor
{
	chassis.setSpeed(vx, vy, vz);
	chassis.getLocation();
}

void chassis_give_speed()
{
	vel_x = chassis._Vx_now * 0.01f;   // cm/s -> m/s
	vel_y = chassis._Vy_now * 0.01f;   // cm/s -> m/s
	vel_z = chassis._W_now;            // rad/s

	odom_x_m = chassis.x * 0.01f;      // cm -> m
	odom_y_m = chassis.y * 0.01f;      // cm -> m
	odom_yaw = chassis.theta;          // rad
}

void chassis_give_vel_only()
{
	// Pinpoint 模式專用：只更新速度，不覆寫 Pinpoint 負責的位置
	vel_x = chassis._Vx_now * 0.01f;
	vel_y = chassis._Vy_now * 0.01f;
	vel_z = chassis._W_now;
}
