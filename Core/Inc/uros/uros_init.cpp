/*
 * uros_init.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */


#include "uros_init.h"
#include <math.h>
#include <string.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

float vx = 0.0 ,vy = 0.0 ,vz = 0.0;

rcl_publisher_t           pose_pub;
nav_msgs__msg__Odometry   pose_msg;
rcl_subscription_t        cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t           arm_pub;
std_msgs__msg__Int32      arm_msg;
rcl_subscription_t        cmd_arm_sub;
std_msgs__msg__Int32      cmd_arm_msg;
rcl_timer_t pose_pub_timer;

// 用于计算积分的变量
static uint32_t last_cmd_vel_time = 0;
static uint32_t last_cmd_arm_time = 0;
static float current_yaw = 0.0f;  // 当前偏航角 (弧度)

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;

agent_status_t status = AGENT_WAITING;


int ping_fail_count = 0;
#define MAX_PING_FAIL_COUNT 5


extern UART_HandleTypeDef USARTx;

void uros_init(void) {
  // Initialize micro-ROS
  rmw_uros_set_custom_transport(
    true,
    (void *) &USARTx,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
  
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
  printf("Error on default allocators (line %d)\n", __LINE__); 
  }
}

void uros_agent_status_check(void) {
  switch (status) {
    case AGENT_WAITING:
      handle_state_agent_waiting();
      break;
    case AGENT_AVAILABLE:
      handle_state_agent_available();
      break;
    case AGENT_CONNECTED:
      handle_state_agent_connected();
      break;
    case AGENT_TRYING:
      handle_state_agent_trying();
      break;
    case AGENT_DISCONNECTED:
      handle_state_agent_disconnected();
      break;
    default:
      break;
  }
}

void handle_state_agent_waiting(void) {
  status = (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) ? AGENT_AVAILABLE : AGENT_WAITING;
}
void handle_state_agent_available(void) {
  uros_create_entities();
  if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
    uros_destroy_entities();     // 沒對到時間就退回，別帶壞 stamp 進 CONNECTED
    status = AGENT_WAITING;
    return;
  }
  status = AGENT_CONNECTED;
}
void handle_state_agent_connected(void) {
  static uint32_t last_sync_ms = 0;
  if(rmw_uros_ping_agent(20, 5) == RMW_RET_OK){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    ping_fail_count = 0; // Reset ping fail count
    uint32_t now = HAL_GetTick();
    if (now - last_sync_ms > 5000) {   // 每 5 秒重新對時，抵銷 HSI 漂移
      rmw_uros_sync_session(100);
      last_sync_ms = now;
    }
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_TRYING;
    }
  }
}
void handle_state_agent_trying(void) {
  if(rmw_uros_ping_agent(50, 10) == RMW_RET_OK){
    status = AGENT_CONNECTED;
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_DISCONNECTED;
      ping_fail_count = 0;
    }
  }
}
void handle_state_agent_disconnected(void) {
  vx = 0.0f; vy = 0.0f; vz = 0.0f;  // 斷線時停車
  uros_destroy_entities();
  status = AGENT_WAITING;
}


void uros_create_entities(void) {
  // 重置积分变量
  last_cmd_vel_time = 0;
  last_cmd_arm_time = 0;
  current_yaw = 0.0f;

  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); // Initialize support structure

  rcl_init_options_fini(&init_options);
  
  rclc_node_init_default(&node, NODE_NAME, "", &support);                       // Initialize node

  rclc_publisher_init_default(                                                  // Initialize publisher for pose
    &pose_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom");

  rosidl_runtime_c__String__assign(&pose_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&pose_msg.child_frame_id, "base_footprint");

  pose_msg.pose.pose.position.x = 0.0;
  pose_msg.pose.pose.position.y = 0.0;
  pose_msg.pose.pose.position.z = 0.0;

  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = 0.0;
  pose_msg.pose.pose.orientation.w = 1.0;

  pose_msg.twist.twist.linear.x = 0.0;
  pose_msg.twist.twist.linear.y = 0.0;
  pose_msg.twist.twist.linear.z = 0.0;
  pose_msg.twist.twist.angular.x = 0.0;
  pose_msg.twist.twist.angular.y = 0.0;
  pose_msg.twist.twist.angular.z = 0.0;

  // odom covariance（2D：未用到的 z/roll/pitch 給大值）
  pose_msg.pose.covariance[0]  = 1e-3;  // x
  pose_msg.pose.covariance[7]  = 1e-3;  // y
  pose_msg.pose.covariance[14] = 1e6;   // z
  pose_msg.pose.covariance[21] = 1e6;   // roll
  pose_msg.pose.covariance[28] = 1e6;   // pitch
  pose_msg.pose.covariance[35] = 1e-2;  // yaw
  pose_msg.twist.covariance[0]  = 1e-3; // vx
  pose_msg.twist.covariance[7]  = 1e-3; // vy
  pose_msg.twist.covariance[35] = 1e-2; // wz

  rclc_publisher_init_default(                                                  // Initialize publisher for pose
    &arm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "robot/arm_status");
  arm_msg.data = -1;

  rmw_uros_set_publisher_session_timeout(                                       // Set session timeout for publisher
    rcl_publisher_get_rmw_handle(&pose_pub),
    10);

  rmw_uros_set_publisher_session_timeout(                                       // Set session timeout for publisher
    rcl_publisher_get_rmw_handle(&arm_pub),
    10);

  rclc_subscription_init_default(                                               // Initialize subscriber for command velocity
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "mecanum/cmd_vel");
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.linear.y = 0.0;
  cmd_vel_msg.linear.z = 0.0;
  cmd_vel_msg.angular.x = 0.0;
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  rclc_subscription_init_default(                                               // Initialize subscriber for arm command
    &cmd_arm_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "robot/cmd_arm");
  cmd_arm_msg.data = -1;


  rclc_timer_init_default(&pose_pub_timer, &support, RCL_MS_TO_NS(50), pose_pub_timer_cb);

  
  rclc_executor_init(&executor, &support.context, 3, &allocator); // Create executor (1 timer + 2 subscriptions)

  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_sub_cb, ON_NEW_DATA); // Add subscriber to executor
  rclc_executor_add_subscription(&executor, &cmd_arm_sub, &cmd_arm_msg, &cmd_arm_sub_cb, ON_NEW_DATA); // Add arm subscriber to executor
  rclc_executor_add_timer(&executor, &pose_pub_timer); // Add timer to executor
}
void uros_destroy_entities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Destroy publisher
  rcl_publisher_fini(&pose_pub, &node);
  rcl_publisher_fini(&arm_pub, &node);

  // Destroy subscriber
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&cmd_arm_sub, &node);

  rcl_timer_fini(&pose_pub_timer);

  // Destroy executor
  rclc_executor_fini(&executor);

  // Destroy node
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void cmd_vel_sub_cb(const void* msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // 检查消息指针是否有效
  if (msg == NULL) {
    return;
  }
  
  cmd_vel_msg = *msg;

  vx = cmd_vel_msg.linear.x * 100.0f;   // m/s -> cm/s（內部運動學用 cm）
  vy = cmd_vel_msg.linear.y * 100.0f;   // m/s -> cm/s
  vz = cmd_vel_msg.angular.z;           // rad/s 不變

//  // 获取当前时间 (毫秒)
//  uint32_t current_time = HAL_GetTick();
//
//  // 计算时间差 (秒)
//  float dt = 0.0f;
//  if (last_cmd_vel_time != 0) {
//	dt = (current_time - last_cmd_vel_time) / 1000.0f;
//	float dx_robot = cmd_vel_msg.linear.x * dt;
//	float dy_robot = cmd_vel_msg.linear.y * dt;
//	//      float cos_yaw = cosf(current_yaw);
//	//      float sin_yaw = sinf(current_yaw);
//	//
//	//      float dx_world = dx_robot * cos_yaw - dy_robot * sin_yaw;
//	//      float dy_world = dx_robot * sin_yaw + dy_robot * cos_yaw;
//
//	  // 更新位置
//	//      pose_msg.pose.pose.position.x += dx_world;
//	//      pose_msg.pose.pose.position.y += dy_world;
//	  // pose_msg.pose.pose.position.z += cmd_vel_msg.linear.z * dt;
//	  pose_msg.pose.pose.position.x += dx_robot;
//	  pose_msg.pose.pose.position.y += dy_robot;
//
//	  // 更新偏航角
//	  current_yaw += cmd_vel_msg.angular.z * dt;
//
//	  // 将偏航角限制在 [-π, π] 范围内
//	  while (current_yaw > M_PI) current_yaw -= 2.0f * M_PI;
//	  while (current_yaw < -M_PI) current_yaw += 2.0f * M_PI;
//
//	  pose_msg.pose.pose.orientation.z = current_yaw;
//
//	  pose_msg.twist.twist.linear.x = cmd_vel_msg.linear.x;
//	  pose_msg.twist.twist.linear.y = cmd_vel_msg.linear.y;
//	  pose_msg.twist.twist.angular.z = cmd_vel_msg.angular.z;
//
////	  rcl_publish(&pose_pub, &pose_msg, NULL);
//  }
//
//  last_cmd_vel_time = current_time;
}

void update_pose(float pos_x, float pos_y, float yaw, float vel_x, float vel_y, float vel_z){
  pose_msg.pose.pose.position.x = pos_x;   // m
  pose_msg.pose.pose.position.y = pos_y;   // m
  pose_msg.pose.pose.position.z = 0.0;

  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = sinf(yaw * 0.5f);   // 繞 Z 的四元數
  pose_msg.pose.pose.orientation.w = cosf(yaw * 0.5f);

  pose_msg.twist.twist.linear.x  = vel_x;  // m/s
  pose_msg.twist.twist.linear.y  = vel_y;  // m/s
  pose_msg.twist.twist.angular.z = vel_z;  // rad/s
}


void pose_pub_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;

  int64_t now_ns = rmw_uros_epoch_nanos();
  if (now_ns < 1500000000000000000LL) {  // 約 2017 年以前 → 視為尚未同步
    return;                               // 不發出近 1970 的 stamp
  }
  pose_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
  pose_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  rcl_publish(&pose_pub, &pose_msg, NULL);
}

void cmd_arm_sub_cb(const void* msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  cmd_arm_msg = *msg;
  uint32_t current_time = HAL_GetTick();

  if(arm_msg.data != cmd_arm_msg.data) {
    if(current_time - last_cmd_arm_time > 2000) {
      arm_msg = cmd_arm_msg;
      rcl_publish(&arm_pub, &arm_msg, NULL);
    }
  }
  else{
    last_cmd_arm_time = current_time;
  }
}
