/** @file demo_flight_control.cpp
*  @version 3.3
*  @date September, 2017
*
*  @brief
*  demo sample of how to use Local position control
*
*  @copyright 2017 DJI. All rights reserved.
*
*/

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include "dji_sdk_demo/system_identification.h"
#include "dji_sdk/dji_sdk.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
//ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlRollPitchYawHeightPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;

float max_angle = 0.1; // in rads
float max_altitude = 1.5;
float max_yaw = 6.28319; // 360 degrees

// log files
std::ofstream position_file;
std::ofstream orientation_file;
std::ofstream controls_file;

int main(int argc, char **argv){
 
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  //ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  //ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  //ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  //ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  //ros::Subscriber initialPosition = nh.subscribe("initial_pos", 1, &initial_pos_callback);

  // Subscribe to the sensor measurements
  ros::Subscriber imu = nh.subscribe("/dji_sdk/imu", 2, &imu_callback);
  ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);

  // Publish the control signal
  //ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
  //ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  position_file.open("/home/ubuntu/results/position.txt");
  orientation_file.open("/home/ubuntu/results/orientation.txt");
  controls_file.open("/home/ubuntu/results/controls.txt");

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  if (takeoff_result){
    printf("READY TO CONTROL \n");
    target_set_state = 1;
  }
 
  ros::spin();
  return 0;
}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat){
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void system_identification(){
  printf("\n-----------------------------------\nSystem identification - (sample k = %d)\n-----------------------------------\n", k);
  k += 1; // sampling step

  if ( k % (1*freq) == 0)
    sign = -1 * sign;
  amplitude = 0.06* sign;
  
  // Publish controls to the drone
  uint8_t flag = (DJISDK::VERTICAL_POSITION   |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY  |
                DJISDK::STABLE_ENABLE);
  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(amplitude); // (desired roll - movement along y-axis)
  controlmsg.axes.push_back(amplitude); // (desired pitch - movement along x-axis)
  controlmsg.axes.push_back(1); // z-cmd (altitude)
  controlmsg.axes.push_back(0); // yaw-cmd (yaw)
  controlmsg.axes.push_back(flag);
  ctrlRollPitchYawHeightPub.publish(controlmsg);

  orientation_file << rpy.x << "," << rpy.y << "," << rpy.z << std::endl;
  controls_file << amplitude << std::endl;
  //orientation_file << curr_imu.angular_velocity.z << std::endl;
  //position_file << curr_pos.x << std::endl;
}

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr &msg){
  curr_pos = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
  curr_imu = *msg;
  if (target_set_state == 1)
    system_identification();
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg){
  local_position = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr &msg){
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr &msg){
  display_mode = msg->data;
}
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg){
  current_atti = msg->quaternion;
  rpy = toEulerAngle(current_atti);
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg){
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr &msg){
  current_gps_health = msg->data;
}

bool takeoff_land(int task){
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if (!droneTaskControl.response.result){
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control(){
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(authority);

  if (!authority.response.result){
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100(){
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if (query.response.version == DJISDK::DroneFirmwareVersion::M100_31){
    return true;
  }

  return false;
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */

bool M100monitoredTakeoff(){
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(4)){
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
    ROS_INFO("Takeoff failed.");
    return false;
  }
  else{
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}
