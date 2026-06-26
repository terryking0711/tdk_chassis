#include <Control_Lib/Pinpoint/Inc/Pinpoint.hpp>
#include <Control_Lib/Pinpoint/Inc/Pinpoint_monitor.hpp>
#include "main.h"
#include "cmsis_os.h"

extern PinpointI2C pinpoint;
extern PinpointI2C::BulkData bd;
extern I2C_HandleTypeDef hi2c1;
float pos_x, pos_y, pos_z;
bool check = 0;
extern float vel_x, vel_y, vel_z;            // 定義於 chassis_monitor.cpp
extern float odom_x_m, odom_y_m, odom_yaw;   // 定義於 chassis_monitor.cpp

 void pinpoint_init(){
 	osDelay(500);
 	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
 		HAL_I2C_DeInit(&hi2c1);
 		osDelay(20);
 		HAL_I2C_Init(&hi2c1);
 	}
	// 需要改變正反向時加入
	// pinpoint.setEncoderDirections(PinpointI2C::EncDir::Reversed, PinpointI2C::EncDir::Forward);
 	pinpoint.Pinpoint_Init();
 }
//void pinpoint_init(){
//    // 增加更長的延遲確保 I2C 裝置穩定
//    osDelay(500);
//
//    // 檢查 I2C 匯流排狀態
//    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
//        HAL_I2C_DeInit(&hi2c1);
//        HAL_I2C_Init(&hi2c1);
//    }
//    check = 1;
//    // 嘗試 ping 裝置
//    if (pinpoint.ping()) {
//        pinpoint.Pinpoint_Init();
//    } else {
//        // I2C 裝置未回應，可能需要重試或報錯
//    }
//}

void pinpoint_monitor(){
	static uint32_t last_ms = 0;
	static uint8_t  fail_cnt = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 50) return;   // 固定 50ms 間隔，不受 I2C 耗時影響
	last_ms = now;

	if (pinpoint.isConnected()) {
		fail_cnt = 0;
		pinpoint.Pinpoint_TaskLoop();
	} else {
		if (++fail_cnt >= 3) {         // 連續 3 次失敗（≈150ms）才嘗試重連
			fail_cnt = 0;
			HAL_I2C_DeInit(&hi2c1);
			osDelay(10);
			HAL_I2C_Init(&hi2c1);
			pinpoint.Pinpoint_Init();
		}
	}
}

void update_pinpoint_pose(){
	odom_x_m = bd.pos_x_mm * 0.001f;    // mm -> m
	odom_y_m = bd.pos_y_mm * 0.001f;    // mm -> m
	odom_yaw = bd.heading_rad;          // rad
	vel_x    = bd.vel_x_mm_s * 0.001f;  // mm/s -> m/s
	vel_y    = bd.vel_y_mm_s * 0.001f;  // mm/s -> m/s
	vel_z    = bd.vel_h_rad_s;          // rad/s
}

