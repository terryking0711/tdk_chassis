/*
 * motor_ctrl.cpp
 *
 *  Created on: Jun 17, 2026
 *      Author: hsuanjung
 */

#include <motor_ctrl.hpp>
int times = 0;
int a = 0;
uint32_t current_cnt;
void MotorController::init(int en_ctrl,int dir_ctrl) {
    HAL_TIM_Encoder_Start(_enc, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(_pwm, _channel);
    _dir_ctrl = dir_ctrl;
    _en_ctrl = en_ctrl;
}

void MotorController::setSpeed(float speed) {

    _targetSpeed = speed;
//    times++;
    ComputePID();
    a++;

//    if (_pidOutput > 80.0) _pidOutput = 80.0;
//    if (_pidOutput < -80.0) _pidOutput = -80.0;
    if (_dir_ctrl == 1){
    	HAL_GPIO_WritePin(_GPIO_INA, _Pin_INA, _pidOutput >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(_GPIO_INB, _Pin_INB, _pidOutput >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }else{
    	HAL_GPIO_WritePin(_GPIO_INA, _Pin_INA, _pidOutput >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    	HAL_GPIO_WritePin(_GPIO_INB, _Pin_INB, _pidOutput >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    _pwmValue = (uint16_t)(fabs(_pidOutput) * PWM_ARR );///  10.0);
    if (_pwmValue < 10) _pwmValue = 0;
    __HAL_TIM_SET_COMPARE(_pwm, _channel, _pwmValue);
//    _targetSpeed = speed;
//    ComputePID();
//    _pwmValue = fabs(_pidOutput) * PWM_ARR;
//
//	if(_pidOutput > 0) HAL_GPIO_WritePin(_dirGPIO, _dirPin, GPIO_PIN_SET);
//	else HAL_GPIO_WritePin(_dirGPIO, _dirPin, GPIO_PIN_RESET);
//
//	__HAL_TIM_SET_COMPARE(_pwm, _channel, (uint16_t)_pwmValue);

}

float MotorController::getSpeed() {
//	updateSpeed();
    return _currentSpeed;
}

float MotorController::ComputePID() {
    updateSpeed();
    _error = _targetSpeed - _currentSpeed;

    _integral += _error * (DT / 1000.0);
    if(_integral >= INTEGRAL_LIMIT) _integral = INTEGRAL_LIMIT;
    else if(_integral <= -INTEGRAL_LIMIT) _integral = -INTEGRAL_LIMIT;


    float derivative = (_error - _lastError) / (DT / 1000.0);

    _pidOutput = (_kp * _error) + (_ki * _integral) + (_kd * derivative);

    // Update last error
    _lastError = _error;
    if(_pidOutput > 1) _pidOutput = 1;
    else if (_pidOutput < -1) _pidOutput = -1;
    return _pidOutput;
}

float MotorController::updateSpeed() {
	cnt = __HAL_TIM_GetCounter(_enc);
	_currentSpeed = (cnt/ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0);
//	int16_t raw_cnt = (int16_t)__HAL_TIM_GetCounter(_enc);
//	_currentSpeed = ((float)raw_cnt / ENCODER_RESOLUTION / REDUCTION_RATIO / 4.0f) / (DT / 1000.0f);
    __HAL_TIM_SET_COUNTER(_enc, 0);
    _currentSpeed *= _en_ctrl;
    return _currentSpeed;
}


