/** @file demo_local_position_control.h
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use local position control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef PROJECT_DEMO_LOCAL_POSITION_CONTROL_H
#define PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#endif //PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <dji_sdk_demo/Pos.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <tf/tf.h>
#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

bool set_local_position();

float target_x;
float target_y;
float target_z;
float target_yaw;
int target_set_state = 0;
float yawinRad;
float rollinRad;
float pitchinRad;

float dx_error=0;
float dy_error=0;

float x_error=0;  // error at k   time
float x_error1=0; // error at k-1 time
float x_error2=0; // error at k-2 time
float y_error=0;  // error at k   time
float y_error1=0; // error at k-1 time
float y_error2=0; // error at k-2 time

// control inputs
float U1 = 0; // total thrust
float U2 = 0; // roll
float U3 = 0; // pitch
float U4 = 0; // yaw

double dt,dt1;
ros::Time start_, end_,s1,e1;

void setTarget(float yaw, float x, float y, float z){
  target_yaw = yaw;
  target_x = x;
  target_y = y;
  target_z = z;
}

void pid_vel_form(); 
void pid_pos_form();
void lqg();

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

void initial_pos_callback(const geometry_msgs::Point::ConstPtr& msg);

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

void local_position_ctrl();