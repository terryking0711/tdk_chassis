# TDK Chassis Firmware — 架構說明

STM32F446 + micro-ROS 麥克納姆四輪底盤。

---

## 本次改動重點（odom 單位 / 方向修正）

### 策略：邊界換算
內部運動學全程維持 **cm / cm/s**，只在「送出 ROS」的邊界換成 SI（m / m/s）。
好處：不需要動積分、PID、encoder 計算，只改兩個點。

### 改動一覽

| 位置 | 改了什麼 | 為何 |
|---|---|---|
| `chassis.cpp` `Mecan_ForwardKinematics()` | Vx/Vy 公式改為共模/差模標準定義，W 符號對齊 | 原公式 Vx/Vy 對調且 W 方向錯誤 |
| `chassis.cpp` `Mecan_InverseKinematics()` | 四輪目標速度符號改為 REP-103 y=左 | 與正運動學對稱 |
| `chassis_monitor.cpp` `chassis_give_speed()` | `vel_*` 乘 0.01 換 m/s；新增 `odom_x_m/y_m/yaw` 由 chassis 積分讀取 | ROS 期待 SI |
| `chassis_monitor.hpp` | 新增 `extern odom_x_m, odom_y_m, odom_yaw` | 讓其他 TU 讀取 |
| `Pinpoint_monitor.cpp` `update_pinpoint_pose()` | mm→m、mm/s→m/s；改寫至 `odom_*` 變數 | 同上 |
| `uros_init.cpp` `cmd_vel_sub_cb()` | 收到的 m/s 乘 100 再寫入 `vx/vy` | 內部運動學用 cm/s |
| `uros_init.cpp` `update_pose()` | 從 yaw 建正確四元數 (sin/cos yaw/2)；補 position.z=0 | 原本只寫 orientation.z = yaw，不是合法四元數 |
| `uros_init.cpp` `uros_create_entities()` | 初始化 pose/twist covariance | 給 EKF 未來使用 |
| `uros_init.cpp` `handle_state_agent_available()` | 時間同步失敗時退回 WAITING | 避免發出 epoch=0 的 stamp |
| `uros_init.cpp` `handle_state_agent_connected()` | 每 5 秒重對時 | 抵銷 HSI 時鐘漂移 |
| `uros_init.cpp` `pose_pub_timer_cb()` | 未同步前不發布 | 避免 ROS TF 收到 1970 時間戳 |
| `rtos-main.c` | Pinpoint 讀取順序修正（先 `pinpoint_monitor()` 再 `update_pinpoint_pose()`）；兩分支都改用 `odom_*` | 原本順序會讀到舊一幀的資料 |

### 全域變數責任表

```
chassis_monitor.cpp  定義:  vel_x, vel_y, vel_z           (m/s, 輪速計算後換算)
                     定義:  odom_x_m, odom_y_m, odom_yaw  (m / rad, 位置積分)

Pinpoint_monitor.cpp 使用:  以上全部（透過 extern）
                     如果 ENABLE_PINPOINT=1，update_pinpoint_pose()
                     會 override odom_* 和 vel_* 為 Pinpoint 的值。

uros_init.cpp        消費:  odom_* / vel_* 透過 update_pose() 填進 nav_msgs/Odometry
```

---

## Pinpoint 是什麼？

**goBILDA Pinpoint Odometry Computer**：一個掛在 I2C 上的專用小板，內建 IMU + 兩個正交光學編碼器 Pod（死輪），在板上即時積分計算 2D 位姿。

### 它做什麼

```
Pinpoint 板（硬體）
  ├── 讀 X-Pod encoder（mm）
  ├── 讀 Y-Pod encoder（mm）
  ├── 讀 IMU（heading）
  └── 板上積分 → 輸出 pos_x_mm, pos_y_mm, heading_rad,
                          vel_x_mm_s, vel_y_mm_s, vel_h_rad_s
```

STM32 這邊的角色：
1. `pinpoint_monitor()` — 定期呼叫 `pinpoint.Pinpoint_TaskLoop()`，透過 I2C 讀取一幀 `BulkData` 存進全域 `bd`。
2. `update_pinpoint_pose()` — 把 `bd` 的值做單位換算（mm→m）後寫進 `odom_*` / `vel_*`。
3. `update_pose()` — 把這些值打包進 `nav_msgs/Odometry` 發布到 ROS。

### 如何啟用

1. 確認 Pinpoint 板透過 I2C1（`hi2c1`，預設地址 `0x31`）接上 STM32。
2. 確認死輪 Pod 安裝方向，必要時在 `Pinpoint_monitor.cpp` 的 `pinpoint_init()` 內呼叫：
   ```cpp
   pinpoint.setEncoderDirections(PinpointI2C::EncDir::Reversed, PinpointI2C::EncDir::Forward);
   ```
3. 根據 Pod 的滾輪直徑調整 `ticks_per_mm`（預設 19.894 tick/mm）：
   ```cpp
   pinpoint.setTicksPerMM(YOUR_VALUE);
   ```
4. 在 `config.h` 將 `ENABLE_PINPOINT` 改為 `1`。

### 何時應該把 ENABLE_PINPOINT 改為 1

| 情境 | 設定 |
|---|---|
| 只測試馬達 / 沒接 Pinpoint 硬體 | `0`（用輪式 odom 積分，精度較低但能跑） |
| Pinpoint 實體已接上並校正完畢 | `1` |
| I2C 出問題或 Pinpoint 掛掉 | 暫時改回 `0`，避免 `HAL_I2C` timeout 卡住主迴圈 |

> 兩種模式的資料出口相同（`odom_x_m/y_m/odom_yaw` + `vel_*`），切換只影響數值來源，不影響 ROS 端任何程式碼。

---

## 未來擴充指引

### 加入 EKF（robot_localization）

EKF 在 ROS 端跑，**韌體端不需要任何改動**，只需確認輸出格式正確：

1. **調整 covariance**：`uros_create_entities()` 已初始化。實車量測後，修改：
   - `pose_msg.pose.covariance[0]`（x）、`[7]`（y）、`[35]`（yaw）
   - `pose_msg.twist.covariance[0]`（vx）、`[7]`（vy）、`[35]`（wz）
   數值越小代表越信任這個感測器。

2. **frame_id**：目前是 `"odom"` → `"base_footprint"`，與 `robot_localization` 預設相符，不需改。

3. **ROS 端**：新增 `robot_localization` 的 `ekf_node`，訂閱 `/odom`，融合 IMU 後發布 `/odometry/filtered`。

---

### 替換 odom 來源（舵輪、差速輪等）

資料流只有一個出口：`update_pose(odom_x_m, odom_y_m, odom_yaw, vel_x, vel_y, vel_z)`。

替換步驟：

1. **新增 `xxx_monitor.cpp`**，在裡面做新的積分邏輯，寫入這六個外部變數：
   ```cpp
   extern float odom_x_m, odom_y_m, odom_yaw;  // 定義於 chassis_monitor.cpp
   extern float vel_x, vel_y, vel_z;
   ```

2. 在 `rtos-main.c` 的 `#else` 分支呼叫你的新函式取代空白區段：
   ```c
   #else
       your_odom_update();   // 寫入 odom_* / vel_*
       update_pose(odom_x_m, odom_y_m, odom_yaw, vel_x, vel_y, vel_z);
   #endif
   ```

3. `update_pose()` 那行**不需要動**。

舵輪範例（偽碼）：
```cpp
void swerve_update_odom(float dt_s) {
    // 1. 讀各輪 encoder + 轉向角，計算底盤速度 (m/s, rad/s)
    vel_x = ...; vel_y = ...; vel_z = ...;
    // 2. 旋轉到世界座標系後積分
    float cos_yaw = cosf(odom_yaw), sin_yaw = sinf(odom_yaw);
    odom_x_m += (vel_x * cos_yaw - vel_y * sin_yaw) * dt_s;
    odom_y_m += (vel_x * sin_yaw + vel_y * cos_yaw) * dt_s;
    odom_yaw += vel_z * dt_s;
}
```

---

### 加入第二個感測器融合（不用 EKF，輕量補丁）

在 `update_pinpoint_pose()` 結尾加入融合，例如用 Pinpoint heading 修正輪式 odom 的航向漂移：

```cpp
// 用 Pinpoint 的 heading 覆蓋 chassis 積分的 yaw（Pinpoint IMU 較不漂移）
odom_yaw = bd.heading_rad;
// 位置仍用 Pinpoint 積分結果
odom_x_m = bd.pos_x_mm * 0.001f;
odom_y_m = bd.pos_y_mm * 0.001f;
```

---

## 已知限制（待實車量測）

- `_W_now` 的分母 `(CHASSIS_WIDTH + CHASSIS_LENGTH)` 實際是否需要乘 0.5，請上車走定點旋轉後比對 ROS 端角速度確認。
- 各輪 encoder 正方向是否與馬達正轉一致，需確認後決定在 `Mecan_ForwardKinematics()` 加負號。
