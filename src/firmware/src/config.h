#ifndef CONFIG_H
#define CONFIG_H

//#define DEBUG

#include <micro_ros_platformio.h>

// Wireless Configuration
#define WIFI_NAME "ROS_NET"
#define WIFI_PASSWORD "RosNet2024!"

// MicroROS Agent Configuration
#define UROS_AGENT_IP IPAddress(192, 168, 1, 3)
#define UROS_AGENT_PORT 8888

// ROS Configuration
#define ROS_NODE "lidar"
#define ROS_NAMESPACE "lidar"
#define ROS_TOPIC_SENDER "measurement"
#define ROS_TOPIC_RECEIVER "speed"
#define ROS_FRAME_ID "laser_frame"

#define MIN_ANGLE 0.0F
#define MIN_ANGLE_RADS (MIN_ANGLE * M_PI / 180.0F)
#define MAX_ANGLE 360.0F
#define MAX_ANGLE_RADS (MAX_ANGLE * M_PI / 180.0F)
#define ANGLE_INCREMENT 0.5F
#define ANGLE_INCREMENT_RADS (ANGLE_INCREMENT * M_PI / 180.0F)
#define MIN_RANGE 0.1F
#define MAX_RANGE 12.0F
#define SCAN_COUNT _abs(MAX_ANGLE - MIN_ANGLE) / ANGLE_INCREMENT

// Configuration for RPLidar
#define RPLIDAR_MOTOR_PIN 4
#define RPLIDAR_SERIAL Serial2
#define RPLIDAR_BAUDRATE 115200

// Setup

void setup_microros()
{
  char wifi_name[] = WIFI_NAME;
  char wifi_password[] = WIFI_PASSWORD;
  set_microros_wifi_transports(wifi_name, wifi_password, UROS_AGENT_IP, UROS_AGENT_PORT);
}

#endif