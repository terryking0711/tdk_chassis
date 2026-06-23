/*
 * motor_control.h
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */

#ifndef INC_CONTROL_LIB_MOTOR_INC_MOTOR_CONFIG_H_
#define INC_CONTROL_LIB_MOTOR_INC_MOTOR_CONFIG_H_

#define WHEEL_RADIUS 0.1 // Wheel radius in meters
#define ENCODER_RESOLUTION 100.0 // Encoder counts per revolution
#define REDUCTION_RATIO  64// Maximum duty cycle for PWM
#define PWM_ARR 1000
#define INTEGRAL_LIMIT 0.5
#define DT 10 // ms


#endif /* INC_CONTROL_LIB_MOTOR_INC_MOTOR_CONFIG_H_ */
