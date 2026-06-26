/*
 * chassis.cpp
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */


#include "chassis.hpp"
#include "motor_config.h"
float Vx_global,Vy_global,dt;
void Chassis::setSpeed(float Vx_goal, float Vy_goal, float W_goal){
	float circle = WHEEL_DIA * PI;
    _Vx_goal = Vx_goal/circle; _Vy_goal = Vy_goal/circle; _W_goal = W_goal/circle;
    Mecan_InverseKinematics();
    _motorFR->setSpeed(_V_FR_goal);
    _motorFL->setSpeed(_V_FL_goal);
    _motorBR->setSpeed(_V_BR_goal);
    _motorBL->setSpeed(_V_BL_goal);
}

void Chassis::getLocation(){
	Mecan_ForwardKinematics();
	dt = DT;
    theta += _W_now * DT/1000;                                              // rad
    Vx_global = _Vx_now * cos(theta) - _Vy_now * sin(theta);   // cm/s
    Vy_global = _Vx_now * sin(theta) + _Vy_now * cos(theta);   // cm/s
    x += (Vx_global * (dt/1000));                                        // cm
    y += (Vy_global * (dt/1000));										// cm
}

void Chassis::Mecan_ForwardKinematics(){
    _V_FR_now = _motorFR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_FL_now = _motorFL->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_BR_now = _motorBR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_BL_now = _motorBL->getSpeed() * WHEEL_DIA * PI;                   // cm/s

    _Vx_now = ( _V_FL_now + _V_FR_now + _V_BL_now + _V_BR_now) / 4.0f;   // cm/s 前進(共模)
    _Vy_now = (-_V_FL_now + _V_FR_now + _V_BL_now - _V_BR_now) / 4.0f;   // cm/s 左移為正
    _W_now  = (-_V_FL_now + _V_FR_now - _V_BL_now + _V_BR_now) / (CHASSIS_WIDTH + CHASSIS_LENGTH); // rad/s
}

void Chassis::Mecan_InverseKinematics(){
//    _V_FR_goal = -_Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//    _V_FL_goal = _Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
//    _V_BR_goal = _Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//    _V_BL_goal = -_Vx_goal + _Vy_goal - _W_goal * _chassis_factor;

//	  _V_FR_goal = _Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//	  _V_FL_goal = _Vx_goal - _Vy_goal - _W_goal * _chassis_factor;
//	  _V_BR_goal = _Vx_goal - _Vy_goal + _W_goal * _chassis_factor;
//	  _V_BL_goal = _Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
	  _V_FL_goal = _Vx_goal - _Vy_goal - _W_goal * _chassis_factor;
	  _V_FR_goal = _Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
	  _V_BL_goal = _Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
	  _V_BR_goal = _Vx_goal - _Vy_goal + _W_goal * _chassis_factor;
}


