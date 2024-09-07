#include "config.h"
#include "macros.h"

#include <Arduino.h>
#include <RPLidar.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/type_utilities.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>

rcl_publisher_t publisher;
rcl_subscription_t subscription;

sensor_msgs__msg__LaserScan scanMessage;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

RPLidar lidar;

// @brief Initialize the scan message
void init_scan_message()
{

  char frame_id[] = ROS_FRAME_ID;

  scanMessage.header.frame_id.data = frame_id;
  scanMessage.angle_min = MIN_ANGLE_RADS;
  scanMessage.angle_max = MAX_ANGLE_RADS;
  scanMessage.angle_increment = ANGLE_INCREMENT_RADS;
  scanMessage.time_increment = 0.0F;
  scanMessage.scan_time = 0.0F;
  scanMessage.range_min = MIN_RANGE;
  scanMessage.range_max = MAX_RANGE;
  scanMessage.ranges.size = SCAN_COUNT;
  scanMessage.ranges.capacity = scanMessage.ranges.size;
  scanMessage.ranges.data = (float*)malloc(scanMessage.ranges.size * sizeof(float));
}

// @brief Initialize the ROS node and allocate topics and necessary resources
void init_uros()
{
  // Initialize micro-ROS with Serial transport
  // set_microros_serial_transport(&Serial1, 115200);

  // Initialize micro-ROS with Wi-Fi transport
  set_microros_wifi_transports(WIFI_NAME, WIFI_PASSWORD, UROS_AGENT_IP, UROS_AGENT_PORT);

  // Creatin Allocator for ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(
    &support, 
    0, 
    NULL, 
    &allocator));

  // Create ROS node
  RCCHECK(rclc_node_init_default(
    &node, 
    ROS_NODE, 
    ROS_NAMESPACE, 
    &support));

  // Create publish topic
  RCCHECK(rclc_publisher_init_default(
      &publisher, 
      &node, 
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      ROS_TOPIC_SENDER));

  // TODO: #1 Create subscription topic for motor speed
  init_scan_message();

}

// @brief Initialize the LiDAR sensor
void init_lidar()
{
  // Setup LiDAR
  lidar.begin(RPLIDAR_SERIAL);

  u_result lidarStatus = lidar.startScan();

  if (IS_FAIL(lidarStatus))
  {
    PRINT_DBG_ARGS("Failed to start scan: error code 0x%x", lidarStatus);
    error_loop();
  }
}

void setup()
{
  Serial.begin(115200);
  PRINT_DBG("Initializing...");

  pinMode(LED_BUILTIN, OUTPUT);

  init_uros();
  init_lidar();
}

void loop()
{

  if(!lidar.isOpen()){
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return;
  }

  u_result scan = lidar.waitPoint();

  // Check for exception from the LIDAR
  if (IS_FAIL(scan))
  {
    PRINT_DBG_ARGS("LiDAR exception %d", scan);
  }

  if (IS_OK(scan))
  {
    // Get the current point
    auto point = lidar.getCurrentPoint();

    // If current point has the start bit set, will be the start of a new scan
    // we send the old scan and start a new one
    if (point.startBit)
    {
      // TODO: #2 Use appropriate UNIX Epoch time for the scan message

      scanMessage.header.stamp.sec = millis() / 1000;
      scanMessage.header.stamp.nanosec = uxr_nanos(); 

      RCSOFTCHECK(rcl_publish(&publisher, &scanMessage, NULL)); // Publish the scan message

      memset(scanMessage.ranges.data, 0, scanMessage.ranges.size * sizeof(float)); // Clear the ranges data
    }

    // Add the new measurement to the array 
    uint16_t idx = round(point.angle / ANGLE_INCREMENT);
    scanMessage.ranges.data[idx] = point.distance / 100.0F;
  }
}