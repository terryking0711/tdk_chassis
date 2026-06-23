/*
 * chassis.hpp
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */

#ifndef INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_HPP_
#define INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_HPP_



#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include <motor_ctrl.hpp>
#include "chassis_config.h"
#include "motor_config.h"

/**********************************************************************/
/*******All the parameter about chassis is in "chassis_config.h"*******/
/**********************************************************************/
extern float Vx_global,Vy_global,dt;

class Chassis {
    public:
        Chassis(MotorController* motorFR, MotorController* motorFL, MotorController* motorBR, MotorController* motorBL):
                _motorFR(motorFR), _motorFL(motorFL), _motorBR(motorBR), _motorBL(motorBL){}
        ~Chassis() = default;

        void setSpeed(float Vx_goal, float Vy_goal, float W_goal);       // Set the speed of chassis
        void getLocation();                                                 // Get the location of the chassis
        void Mecan_ForwardKinematics();                                     // Compute the forward kinematics of mecanum chassis
        void Mecan_InverseKinematics();                                     // Compute the inverse kinematics of mecanum chassis
        float x = INIT_X, y = INIT_Y, theta = INIT_THETA;
        MotorController* _motorFR;
        MotorController* _motorFL;
        MotorController* _motorBR;
        MotorController* _motorBL;
        float _Vx_now = 0, _Vy_now = 0, _W_now = 0;
    private:

        float _V_FR_goal = 0, _V_FL_goal = 0, _V_BR_goal = 0, _V_BL_goal = 0;
        float _Vx_goal = 0, _Vy_goal = 0, _W_goal = 0;
        float _chassis_factor = (CHASSIS_LENGTH + CHASSIS_WIDTH) / (PI * WHEEL_DIA);
        float _V_FR_now = 0, _V_FL_now = 0, _V_BR_now = 0, _V_BL_now = 0;
};


#ifdef __cplusplus
}
#endif



#endif /* INC_CONTROL_LIB_CHASSIS_INC_CHASSIS_HPP_ */
