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

#ifndef PROJECT_HOVER_H
#define PROJECT_HOVER_H

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
#include <string>
#include <iostream>
#include <fstream>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define POS_THRESH 0.2
bool set_local_position();
/// variables

sensor_msgs::Imu curr_imu;
geometry_msgs::Point initial_pos;
geometry_msgs::PointStamped uwb_pos;
geometry_msgs::Point prev_pos;

geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;

geometry_msgs::Quaternion current_atti;

dji_sdk_demo::Pos desired_pos;
dji_sdk_demo::Pos curr_pos;
// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;

float target_offset_x;
float target_offset_y;
float target_offset_z;
float target_yaw;
int target_set_state = 0;
float yawinRad;
float rollinRad;
float pitchinRad;

float dx_error=0;
float dy_error=0;

float rolldes;//desired roll
float pitchdes;//desired pitch
float pitch_prev=0;
float pitch=0;
float roll=0;
float roll_prev=0;
float x_error=0;  // error at k   time
float x_error1=0; // error at k-1 time
float x_error2=0; // error at k-2 time
float y_error=0;  // error at k   time
float y_error1=0; // error at k-1 time
float y_error2=0; // error at k-2 time

geometry_msgs::Vector3 rpy;// drone euler angles
std::string fileid="0";
//double dt,dt1;
//ros::Time start_, end_,s1,e1;

  std::ofstream imuAcc;
  std::ofstream imuG;
  std::ofstream imuRPY;
  std::ofstream uwb;
  std::ofstream comm;
  std::ofstream err;

void setTarget(float x, float y, float z, float yaw)
{
  target_offset_x = x;
  target_offset_y = y;
  target_offset_z = z;
  target_yaw      = yaw;
}

void open_data_files(std::string fid){
  uwb.open("position"+fid+".txt");
  comm.open("commands"+fid+".txt");
}

void write_data_flight(){
  comm<<pitch<<" "<<pitchdes<<" "<<rpy.y<<" "<<roll<<" "<<rolldes<<" "<<rpy.x<<" "<<rpy.z<<std::endl ;
  uwb<<curr_pos.x<<" "<<curr_pos.y<<" "<<curr_pos.z<<" "<<std::endl;
}

void control();

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