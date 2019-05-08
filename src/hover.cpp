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
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk_demo/hover.h" // Includes also the KalmanFilter, MCCKalmanFilter and HinfFilter class headers

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

sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Imu curr_imu;

geometry_msgs::PointStamped local_position;
geometry_msgs::Point initial_pos;
geometry_msgs::PointStamped uwb_pos;
geometry_msgs::Point prev_pos;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 rpy;

//dji_sdk_demo::Pos desired_pos;
dji_sdk_demo::Pos curr_pos;

float kp = 0.1; // proportional gain P
float ki = 0.005; // integral gain I
float kd = 4.5; // derivative gain D
float max_angle = 0.1;
float max_altitude = 1.5;

std::ofstream imuAcc;
std::ofstream imuG;
std::ofstream imuRPY;

// log files
std::ofstream position_file;
std::ofstream orientation_file;
std::ofstream states_file;
std::ofstream controls_file;
std::ofstream gains_file;

ros::Publisher ctrlRollPitchYawHeightPub;
ros::Publisher plotpos;

std::chrono::high_resolution_clock::time_point t1;
std::chrono::high_resolution_clock::time_point t2;
std::chrono::high_resolution_clock::time_point t3;
std::chrono::high_resolution_clock::time_point t4;

KalmanFilter kf(n, m);
// MCCKalmanFilter kf(n, m);
// HinfFilter kf(n, m);

int main(int argc, char **argv){
  //LQR lqr(n,j);

  x0 << 0,0,0,0,0,0,2.1,0,2.6,0,0.8,0; // remember to update accordingly every experiment
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
  L << 8.77e-13,3.94-14,-5.06e-12,-3.70e-13,-2.49e-13,-8.13e-14,4.94e-13,2.20e-12,3.13e-13,8.33e-14,-92.30,-35.99,
       7.85,1.43,-6.67e-14,-2.87e-15,-5.36e-16,-4.19e-15,3.85e-14,2.27e-14,2.87,2.33,-3.07e-14,-1.22e-15,
       -4.72e-14,-2.77e-15,7.85,1.43,-1.57e-16,-7.89e-16,-2.87,-2.33,-2.81e-14,-1.71e-14,1.47e-13,1.12e-14,
       -2.77e-14,-2.23e-15,-3.20e-14,-5.52e-16,0.96,1.11,3.16e-14,1.18e-14,-2.27e-14,-1.65e-14,1.86e-15,1.23e-15;
  
  // LQR weighting matrices
  Q.setIdentity(n, n);
  R.setIdentity(j, j);
  Q(7,7)=10;
  Q(9,9)=10;
  Q(11,11)=10;
  R(0,0)=0.001;
  K.setZero(j, n);
  // Kalman filter covariance matrices
  In.setIdentity(n, n);
  Im.setIdentity(m,m);
  P0=10*In;
  Qn=5*In;
  Rn=5*Im;
  // Kalman Filter initialization
  kf.setFixed(A, C, Qn, Rn);
  kf.setInitial(x0, P0);
  // Reference tracking
  A_cl = (A-B*L);
  G = -B.inverse()*A_cl*C.inverse();
  
  t2 = std::chrono::high_resolution_clock::now();
  t4 = std::chrono::high_resolution_clock::now();
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

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  //imuAcc.open("distacc.txt");
  //imuG.open("accels.txt");
  position_file.open("position.txt");
  orientation_file.open("orientation.txt");
  states_file.open("states.txt");
  controls_file.open("controls.txt");
  gains_file.open("gains.txt");

  t1 = std::chrono::high_resolution_clock::now();
  t2 = std::chrono::high_resolution_clock::now();
  ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);

  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  t2 = std::chrono::high_resolution_clock::now();
  t4 = std::chrono::high_resolution_clock::now();
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

  x_error = target_x - curr_pos.x;
  y_error = target_y - curr_pos.y;
}

void pid_vel_form(){
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

  x_error2 = x_error1;
  x_error1 = x_error;
  y_error2 = y_error1;
  y_error1 = y_error;
}

void lqg(){
  printf("\n-------------------------\nLQG control - (sample k = %d)\n-------------------------\n", k);
  Eigen::VectorXf y(m); // Measurement vector
  Eigen::VectorXf ref(m); // Reference vector
  
  ref << target_yaw,target_x,target_y,target_z;
  y << rpy.z, curr_pos.x, curr_pos.y, curr_pos.z;
  
  // LQR tracking control
  u << G*ref - L*kf.X;
  
  // State estimation using Kalman Filter
  kf.predict(u); //Predict phase, if the system is controlled then pass an input vector u as an argument
  kf.correct(y); //Correction phase

  // Bound angles and height for safety and linerized operating point
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

  // Publish controls to the drone
  //sensor_msgs::Joy controlmsg;
  //controlmsg.axes.push_back(U2); // roll
  //controlmsg.axes.push_back(U3); // pitch
  //controlmsg.axes.push_back(U1); // total thrust
  //controlmsg.axes.push_back(U4); // yaw
  //ctrlRollPitchYawHeightPub.publish(controlmsg);

  std::cout << kf.X << "";
  printf("Roll: %f\tPitch: %f\tYaw: %f\n", rpy.x, rpy.y, rpy.z); // yaw was multiplied by * 57.29
  printf("PosX: %f\tPosY: %f\tPosZ: %f\n\n", curr_pos.x, curr_pos.y, curr_pos.z); // yaw was multiplied by * 57.29
 
  orientation_file << rpy.x << "," << rpy.y << "," << rpy.z << std::endl;
  position_file << curr_pos.x << "," << curr_pos.y << "," << curr_pos.z << std::endl;
  states_file << kf.X[0]<<","<<kf.X[1]<<","<<kf.X[2]<<","<<kf.X[3]<<","<<kf.X[4]<<","<<kf.X[5]<<","<<kf.X[6]<<","<<kf.X[7]<<","<<kf.X[8]<<","<<kf.X[9]<<","<<kf.X[10]<<","<<kf.X[11] << std::endl;
  controls_file << U1 << "," << U2 << "," << U3 << "," << U4 << std::endl;

  k += 1; // sampling step
}

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr &msg){
  curr_pos = *msg;

  int elapsed_time = ros::Time::now().toNSec(); // - posi.nses;

  target_set_state = 1; // remember to delete line for normal experiments
  if (target_set_state == 1) {
    setTarget(0.0,3.0,4.0,1.2); // target_yaw, target_x, target_y, target_z
    lqg();  // pid_vel_form(), pid_pos_form()
    //comm<<_span.count()<<std::endl;
    //elapsed_time=ros::Time::now().toNSec()-elapsed_time+curr_pos.time;
    //uwb << curr_pos.x << "," << curr_pos.y << "," << curr_pos.z << "," << elapsed_time << std::endl;
  } 
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
  curr_imu = *msg;
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

using namespace std;
// Constructor:
KalmanFilter::KalmanFilter(int _n,  int _m) {
	n = _n;
	m = _m;
}

// Set Fixed Matrix
void KalmanFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Fixed Matrix
void KalmanFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Initial Matrix
void KalmanFilter::setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

// Do prediction based of physical system (No external input)
void KalmanFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

// Do prediction based of physical system (with external input)  U: Control vector
void KalmanFilter::predict( Eigen::VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

// Correct the prediction, using mesaurement Y: mesaure vector
void KalmanFilter::correct ( Eigen::VectorXf Y ) {
  K = ( P * C.transpose() ) * ( C * P * C.transpose() + R).inverse();
  X = X + K*(Y - C * X);
  P = (I - K * C) * P;
  X0 = X;
  P0 = P;
}

// Constructor:
MCCKalmanFilter::MCCKalmanFilter(int _n,  int _m, int _sigma) {
	n = _n;
	m = _m;
  sigma = _sigma;
}

// Set Fixed Matrix
void MCCKalmanFilter::setFixed( MatrixXf _A, MatrixXf _C, MatrixXf _Q, MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Fixed Matrix
void MCCKalmanFilter::setFixed( MatrixXf _A, MatrixXf _B, MatrixXf _C, MatrixXf _Q, MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Initial Matrix
void MCCKalmanFilter::setInitial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

// Do prediction based of physical system (No external input)	 
void MCCKalmanFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

// Do prediction based of physical system (with external input) U: Control vector	 
void MCCKalmanFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

// Correct the prediction, using mesaurement  Y: mesaure vector
void MCCKalmanFilter::correct ( VectorXf Y ) {
  R_inv = R.inverse();
  cov_P = P;
  innov_num = Y - C*X_old;
  innov_den = X_old - A*X;
  norm_num = norm(innov_num);
  norm_den = norm(innov_den);
  // norm_innov = sqrt((innov).'*invers_R*(innov));
  Gkernel = exp(-(pow(norm_num,2)) /(2*pow(sigma,2)))/exp(-(pow(norm_den,2))/(2*pow(sigma,2)));
  K = (cov_P.inverse() + Gkernel * C.transpose()*R_inv*C).inverse() * Gkernel * C.transpose()*R_inv;
  X = X + K *(innov_num);
  P = (I - K*C) * cov_P * (I - K*C).transpose() + (K*R*K.transpose());

  X0 = X;
  P0 = P;
}

// Constructor:
HinfFilter::HinfFilter(int _n,  int _m, int _sigma) {
	n = _n;
	m = _m;
  sigma = _sigma;
}

// Set Fixed Matrix
void HinfFilter::setFixed( MatrixXf _A, MatrixXf _C, MatrixXf _Q, MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Fixed Matrix
void HinfFilter::setFixed( MatrixXf _A, MatrixXf _B, MatrixXf _C, MatrixXf _Q, MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

// Set Initial Matrix 
void HinfFilter::setInitial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

// Do prediction based of physical system (No external input)	 
void HinfFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

// Do prediction based of physical system (with external input)  U: Control vector	 
void HinfFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

// Correct the prediction, using mesaurement  Y: mesaure vector
void HinfFilter::correct ( VectorXf Y ) {
  R_inv = R.inverse();
  P_old = P;
  innov_num = Y - C*X;
  norm_num = norm(innov_num);
  K = P_old * (I-theta*P_old+C.transpose()*R_inv*C*P_old).inverse()*C.tranpose()*R_inv;
  X = X + K *(innov_num);
  P = P_old * (I - theta*P_old + C.transpose() * R_inv * C * P_old).inverse();

  X0 = X;
  P0 = P;
}