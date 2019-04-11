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

#include "dji_sdk_demo/hover.h"
#include "dji_sdk/dji_sdk.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <Eigen/Dense>
#include "LinearQuadraticRegulator.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Imu curr_imu;
geometry_msgs::Point initial_pos;
geometry_msgs::PointStamped uwb_pos;
geometry_msgs::Point prev_pos;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 rpy;

//dji_sdk_demo::Pos desired_pos;
dji_sdk_demo::Pos curr_pos;

int k = 0; // Sampling step
int n = 12; // Number of states
int m = 4; // Number of measurements
int j = 4; // Number of inputs
int T = 4; // Sliding window size
float kp = 0.1; // proportional gain P
float ki = 0.005; // integral gain I
float kd = 4.5; // derivative gain D
float max_angle = 0.1;
float max_altitude = 1.5;

std::ofstream imuAcc;
std::ofstream imuG;
std::ofstream imuRPY;
std::ofstream uwb;
std::ofstream comm;
std::ofstream err;

ros::Publisher ctrlRollPitchYawHeightPub;
ros::Publisher plotpos;

using namespace std::chrono;
high_resolution_clock::time_point t1;
high_resolution_clock::time_point t2;
high_resolution_clock::time_point t3;
high_resolution_clock::time_point t4;

Eigen::MatrixXf A(n, n); // System dynamics matrix
Eigen::MatrixXf B(n, j); // Control input matrix
Eigen::MatrixXf C(m, n); // Output matrix
Eigen::MatrixXd Q(n, n); // State penalize matrix
Eigen::MatrixXd R(j, j); // Input penalize matrix
Eigen::MatrixXf Qn(n, n); // Process noise covariance
Eigen::MatrixXf Rn(m, m); // Measurement noise covariance 
Eigen::MatrixXf P0(n, n); // Estimate error covariance
Eigen::MatrixXf In(n, n); // Identity matrix
Eigen::MatrixXf Im(m, m); // Identity matrix
Eigen::VectorXf x0(n); // Initial states
Eigen::MatrixXf K(j, n); // Controller gain matrix
Eigen::MatrixXf A_cl(n, n); // Closed loop matrix
Eigen::MatrixXf G(j, m); // Gain filter tracking matrix
Eigen::MatrixXd Adbl(n, n); // System dynamics matrix (double)
Eigen::MatrixXd Bdbl(n, j); // Control input matrix (double)

x0 << 0,0,0,0,0,0,0,0,0,0,0,0;

// Discrete LTI projectile motion, measuring position only
A <<  1,0.01,0,0,0,0,0,0,0,0,0,0,
      0,1,0,0,0,0,0,0,0,0,0,0,
      0,0,1,0.01,0,0,0,0,0,0,0,0,
      0,0,0,1,0,0,0,0,0,0,0,0,
      0,0,0,0,1,0.01,0,0,0,0,0,0,
      0,0,0,0,0,1,0,0,0,0,0,0,
      0,0,-0.0004905,-0.000001635,0,0,1,0.01,0,0,0,0,
      0,0,-0.0981,-0.0004905,0,0,0,1,0,0,0,0,
      0.0004905,0.000001635,0,0,0,0,0,0,1,0.01,0,0,
      0.0981,0.0004905,0,0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,0,0,0,1,0.01,
      0,0,0,0,0,0,0,0,0,0,0,1;

B <<  0,0.0006349,0,0,
      0,0.1269809,0,0,
      0,0,0.0006349,0,
      0,0,0.1269809,0,
      0,0,0,0.00031742,
      0,0,0,0.063484,
      0,0,-0.0000000519,0,
      0,0,-0.00002076,0,
      0,0.0000000519,0,0,
      0,0.00002076,0,0,
      -0.00002083,0,0,0,
      -0.00417,0,0,0;

C <<  0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;

Q.setIdentity(n, n);
R.setIdentity(j, j);
Q(7,7)=10;
Q(9,9)=10;
Q(11,11)=10;
R(0,0)=0.001;
K.setZero(j, n);

// For Kalman filter
In.setIdentity(n, n);
Im.setIdentity(m,m);
P0=10*In;
Qn=5*In;
Rn=5*Im;

int main(int argc, char **argv){
  /* Create and initialize the Kalman Filter*/
  KalmanFilter kf(n, m);
  kf.setFixed(A, C, Qn, Rn);
  kf.setInitial(x0, P0);

  /* Create The LQR controller */
  LQR lqr(n,j);
  Adbl = A.cast <double> (); // Matrix of floats to double.
  Bdbl = B.cast <double> (); // Matrix of floats to double.
  lqr.setFixed(Adbl,Bdbl,Q,R);
  lqr.solver();

  // Reference tracking gain filter
  K = lqr.Klqr.cast <float> (); // Matrix of doubles to floats.
  A_cl = (A-B*K);
  G = -B.inverse()*A_cl*C.inverse();

  t2 = high_resolution_clock::now();
  t4 = high_resolution_clock::now();
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node

  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber imu = nh.subscribe("/dji_sdk/imu", 2, &imu_callback);

  //ros::Subscriber initialPosition = nh.subscribe("initial_pos", 1, &initial_pos_callback);

  // Publish the control signal

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  //imuAcc.open("distacc.txt");
  //imuG.open("accels.txt");
  uwb.open("position.txt");
  comm.open("attitude.txt");
  err.open("gains.txt");
  err << kp << " " << ki << " " << kd << " " << std::endl;

  t1 = high_resolution_clock::now();
  t2 = high_resolution_clock::now();
  ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);

  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  t2 = high_resolution_clock::now();
  t4 = high_resolution_clock::now();
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

void pid_pos_form(){
  VectorXf u(j); // Control input vector
  u[0] = 0.0; // zero yaw
  u[1] = kp * (target_y - curr_pos.y) + kd * (target_y - curr_pos.y - y_error);
  u[2] = kp * (target_x - curr_pos.x) + kd * (target_x - curr_pos.x - x_error);
  u[3] = 1.2; // constant altitude z

  U1 = u[3];
  if (fabs(u[1]) >= max_angle)
    U2 = (u[1] > 0) ? max_angle : -1 * max_angle;
  else
    U2 = u[1]; 
  if (fabs(u[2]) >= max_angle)
    U3 = (u[2] > 0) ? max_angle : -1 * max_angle;
  else
    U3 = u[2];
  U4 = u[0];

  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(U2); // roll
  controlmsg.axes.push_back(U3); // pitch
  controlmsg.axes.push_back(U1); // total thrust
  controlmsg.axes.push_back(U4); // yaw
  ctrlRollPitchYawHeightPub.publish(controlmsg);

  comm << pitch << " " << U3 << " " << roll << " " << U2 << std::endl;

  x_error = target_x - curr_pos.x;
  y_error = target_y - curr_pos.y;
}

void pid_vel_form(){
  VectorXf u(j); // Control input vector
  
  x_error = target_x - curr_pos.x;
  y_error = target_y - curr_pos.y;

  u[0] = 0.0; // zero yaw
  u[1] = U2 + kp * (y_error - y_error1) + ki * y_error + kd * (y_error - 2 * y_error1 + y_error2);
  u[2] = U3 + kp * (x_error - x_error1) + ki * x_error + kd * (x_error - 2 * x_error1 + x_error2);
  u[3] = 1.2; // constant altitude z

  U1 = u[3];
  if (fabs(u[1]) >= max_angle)
    U2 = (u[1] > 0) ? max_angle : -1 * max_angle;
  else
    U2 = u[1]; 
  if (fabs(u[2]) >= max_angle)
    U3 = (u[2] > 0) ? max_angle : -1 * max_angle;
  else
    U3 = u[2];
  U4 = u[0];

  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(U2); // roll
  controlmsg.axes.push_back(U3); // pitch
  controlmsg.axes.push_back(U1); // total thrust
  controlmsg.axes.push_back(U4); // yaw
  ctrlRollPitchYawHeightPub.publish(controlmsg);
  comm << pitch << " " << U3 << " " << roll << " " << U2 << std::endl;

  x_error2 = x_error1;
  x_error1 = x_error;
  y_error2 = y_error1;
  y_error1 = y_error;
}

void lqg(){
  VectorXf y(m); // Measurement vector
  VectorXf u_init(j); // Control input vector
  VectorXf ref(m); // Reference vector

  if (k==0)
    u_init << 0,0,0,0;

  k += 1; // sampling step - increases one step every lqg() function call

  ref << target_yaw,target_x,target_y,target_z;
  y << rpy.z, curr_pos.x, curr_pos.y, curr_pos.z;

  if (!u_init){
    VectorXf u(j); // Control input vector
    u = u_init;
    u_init = 1;       
  }

  // State estimation using Kalman Filter
  kf.predict(u); //Predict phase, if the system is controlled then pass an input vector u as an argument
  kf.correct(y); //Correction phase

  // LQR tracking control
  u << G*ref - K*kf.X;

  if (fabs(u[3]) >= max_altitude)
    U1 = max_altitude;
  else
    U1 = u[3];
  if (fabs(u[1]) >= max_angle)
    U2 = (u[1] > 0) ? max_angle : -1 * max_angle;
  else
    U2 = u[1];
  if (fabs(u[2]) >= max_angle)
    U3 = (u[2] > 0) ? max_angle : -1 * max_angle;
  else
    U3 = u[2];
  U4 = u[0];

  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(U2); // roll
  controlmsg.axes.push_back(U3); // pitch
  controlmsg.axes.push_back(U1); // total thrust
  controlmsg.axes.push_back(U4); // yaw
  ctrlRollPitchYawHeightPub.publish(controlmsg);
  comm << roll << " " << U2 << " " << pitch << " " << U3 << << " " << yaw << " " << U4 << std::endl;
}

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr &msg){
  t1 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
  t2 = high_resolution_clock::now();
  curr_pos = *msg;
  //ROS_INFO("I heard: [%d]",( posi.nses));

  int elapsed_time = ros::Time::now().toNSec(); // - posi.nses;
  //-1 pitch opposite sign

  printf("Roll: %f\nPitch: %f\nYaw: %f\n", rpy.x, rpy.y, rpy.z); // yaw was multiplied by * 57.29
  printf("PosX: %f\nPosY: %f\nPosZ: %f\n\n", curr_pos.x, curr_pos.y, curr_pos.z); // yaw was multiplied by * 57.29

  if (target_set_state == 1) {
    //pid_vel_form();
    //pid_pos_form(); 
    setTarget(0.0,3.0,4.0,1.2); // target_yaw, target_x, target_y, target_z
    lqg();
    //comm<<_span.count()<<std::endl;
    //elapsed_time=ros::Time::now().toNSec()-elapsed_time+curr_pos.time;
    uwb << curr_pos.x << " " << curr_pos.y << " " << curr_pos.z << " " << elapsed_time << std::endl;
  } 
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
  curr_imu = *msg;
  /*
    t1 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
    t2 = high_resolution_clock::now();

  //  auto duration = duration_cast<microseconds>( t2 - t1 ).count()
  imuG<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<time_span.count()<<std::endl;
  //imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
  xspeed+=((curr_imu.linear_acceleration.x+ax_prev)/2.0)*time_span.count();
  yspeed+=((curr_imu.linear_acceleration.y+ay_prev)/2.0)*time_span.count();

  dximu+=((xspeed_prev+xspeed)/2.0)*time_span.count();  
  dyimu+=((yspeed_prev+yspeed)/2.0)*time_span.count(); 
  imuAcc<<dximu<<" "<<dyimu<<" "<<xspeed<<" "<<yspeed<<std::endl;
  //imuAcc<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<dt<<std::endl;
  //imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
  xspeed_prev=xspeed;
  yspeed_prev=yspeed;
  ax_prev=curr_imu.linear_acceleration.x;
  ay_prev=curr_imu.linear_acceleration.y;
  */
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
  while (ros::Time::now() - start_time < ros::Duration(10)){
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
    ROS_INFO("Takeoff failed.");
    //return false;
  }
  else{
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}