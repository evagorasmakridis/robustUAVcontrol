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
#include "dji_sdk_demo/hover.h"
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

float kp = 0.1; // proportional gain P
float ki = 0.005; // integral gain I
float kd = 4.5; // derivative gain D
float max_angle = 0.1; // in rads

// log files
std::ofstream position_file;
std::ofstream cct_file;
std::ofstream variance_file;
std::ofstream orientation_file;
std::ofstream states_file;
std::ofstream controls_file;
std::ofstream gains_file;

KalmanFilter estimator(n, m);
//MCCKalmanFilter estimator(n, m, sigma);
// HinfFilter kf(n, m);

int main(int argc, char **argv){
  // Discrete LTI 100Hz (imported from Matlab model)
  u << 0, 0; // initial input
  
  A << 0.992, 0.019, 0, 0, 0, 0, 0, 0,
       -0.745, 0.864, 0, 0, 0, 0, 0, 0,
       0, 0, 0.993, 0.019, 0, 0, 0, 0,
       0, 0, -0.715, 0.871, 0, 0, 0, 0,
       0, 0, 0.002, 0, 1, 0.02, 0, 0,
       0, 0, 0.196, 0.002, 0, 1.007, 0, 0,
       -0.002, -0, 0, 0, 0, 0, 1, 0.02,
       -0.196, -0.002, 0, 0, 0, 0, 0, 1.007;

  B << 0.007, 0,
       0.702, 0,
       0, 0.007,
       0, 0.676,
       0, 0,
       0, 0,
       0, 0,
       0, 0;

  C << 1, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 0;

  Cc << 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0;

//  L << 0.2770, 0.0380, 0, 0, 0, 0, -0.0310, -0.1460,
//       0, 0, 0.2770, 0.0390, 0.0310, 0.1450, 0, 0;
  L << 0.746,0.091,0,0,-0,0,-0.097,-0.255,
       0,0,0.754,0.096,0.097,0.255,-0,-0;

  // LQR weighting matrices
  // Q.setIdentity(n, n);
  // R.setIdentity(j, j);
  
  // Kalman filter initial covariance matrices
  In.setIdentity(n,n);
  Im.setIdentity(m,m);
  P0=4*In;
  Qn=0.1*In;
  Rn=1*Im;
  Qn(0,0) = 0.001;
  Qn(2,2) = 0.001;
  Qn(4,4) = 1;
  Qn(6,6) = 0.1;
  Rn(0,0) = 0.003;
  Rn(1,1) = 0.002;
  Rn(2,2) = 2.4;
  Rn(3,3) = 0.5;
 
  ref << setTarget(5.0,2.0,1.0,0.0); // x,y,z,psi
 
  // Reference tracking
  Lc = -(Cc*(In-A-B*L).inverse()*B).inverse();
  servo_compensator = Lc*ref;

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
  ros::Subscriber uwbPosition = nh.subscribe("/uwb_pos", 1, &uwb_position_callback);

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
  variance_file.open("/home/ubuntu/results/variance.txt");
  orientation_file.open("/home/ubuntu/results/orientation.txt");
  states_file.open("/home/ubuntu/results/states.txt");
  controls_file.open("/home/ubuntu/results/controls.txt");
  gains_file.open("/home/ubuntu/results/gains.txt");
  cct_file.open("/home/ubuntu/results/control_computation_time.txt");

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  if (takeoff_result){
    printf("READY TO CONTROL \n");
    target_set_state = 1;
  }
  ros::spin();
  return 0;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat){
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void pid_pos_form(){
  printf("\n-----------------------------------\nPID control - (sample k = %d)\n-----------------------------------\n", k);
  /*
  k += 1; // sampling step
 
  u[0] = kp * (target_x - curr_pos.x) + kd * (target_x - curr_pos.x - x_error) - 0.006;
  u[1] = kp * (target_y - curr_pos.y) + kd * (target_y - curr_pos.y - y_error) - 0.007;
  u[2] = 0.8; // height
  u[3] = 0; // yaw angle
  
  // Bound angles and height for safety and linerized operating point
  if (fabs(u[0]) >= max_angle)
    pitch_cmd = (u[0] > 0) ? max_angle : -1 * max_angle;
  else
    pitch_cmd = u[0];
  
  if (fabs(u[1]) >= max_angle)
    roll_cmd = (u[1] > 0) ? max_angle : -1 * max_angle;
  else
    roll_cmd = u[1];

  if (fabs(u[2]) >= max_yaw)
    yaw_cmd = (u[2] > 0) ? max_yaw : -1 * max_yaw;

  if (fabs(u[3]) >= max_altitude)
    z_cmd = (u[3] > 0) ? max_altitude : -1 * max_altitude;

  // Publish controls to the drone
  uint8_t flag = (DJISDK::VERTICAL_POSITION   |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY  |
                DJISDK::STABLE_ENABLE);
  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(roll_cmd); // (desired roll - movement along y-axis)
  controlmsg.axes.push_back(pitch_cmd); // (desired pitch - movements along x-axis)
  controlmsg.axes.push_back(1); // z-cmd (desired height in meters)
  controlmsg.axes.push_back(0); // (desired yaw rate)
  controlmsg.axes.push_back(flag);
  ctrlRollPitchYawHeightPub.publish(controlmsg);

  x_error = target_x - curr_pos.x;
  y_error = target_y - curr_pos.y;
 
  orientation_file << rpy.x << "," << rpy.y << "," << rpy.z << std::endl;
  position_file << curr_pos.x << "," << curr_pos.y << "," << curr_pos.z << std::endl;
  controls_file << pitch_cmd << "," << roll_cmd << "," << yaw_cmd << "," << z_cmd << std::endl;
  */
}

void lqg(){
  printf("\n-----------------------------------\nLQG control - (sample k = %d)\n-----------------------------------\n", k);
  y << rpy.x, rpy.y, curr_pos.x, curr_pos.y;
  
  if (k==0){
    // Initial states
    x0 << rpy.x, 0, rpy.y, 0, curr_pos.x, 0, curr_pos.y, 0; // phi,phid,theta,thetad,psi,psid,z,zd,x,xd,y,yd
    // Kalman filter initialization
    estimator.setFixed(A, B, C, Qn, Rn);
    estimator.setInitial(x0, P0);
    // Initial control input
    u << servo_compensator - L*x0;
  }
  else {
    // LQG reference tracking control
    u << servo_compensator - L*estimator.X; 
  }

  // Rn = sliding_window(y);
  
  // State estimation
  estimator.predict(u);
  estimator.correct(y);
 
  // Bound angles and height for safety and linerized operating point
  if (fabs(u[0]) >= max_angle)
    roll_cmd = (u[0] > 0) ? max_angle : -1 * max_angle;
  else
    roll_cmd = u[0];
  
  if (fabs(u[1]) >= max_angle)
    pitch_cmd = (u[1] > 0) ? max_angle : -1 * max_angle;
  else
    pitch_cmd = u[1];

  // Publish controls to the drone
  uint8_t flag = (DJISDK::VERTICAL_POSITION   |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY  |
                DJISDK::STABLE_ENABLE);
  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(roll_cmd); // (desired roll - movement along y-axis)
  controlmsg.axes.push_back(pitch_cmd); // (desired pitch - movements along x-axis)
  controlmsg.axes.push_back(1); // z-cmd (desired height in meters)
  controlmsg.axes.push_back(0); // (desired yaw rate)
  controlmsg.axes.push_back(flag);
  ctrlRollPitchYawHeightPub.publish(controlmsg);

  orientation_file << rpy.x << "," << rpy.y << "," << rpy.z << std::endl;
  position_file << curr_pos.x << "," << curr_pos.y << "," << curr_pos.z << std::endl;
  //variance_file << Rn(0,0) << "," << Rn(1,1) << "," << Rn(2,2) << "," << Rn(3,3) << std::endl;
  states_file << estimator.X[0] << "," << estimator.X[1] << "," << estimator.X[2] << "," << estimator.X[3] << "," << estimator.X[4] << "," << estimator.X[5] << "," << estimator.X[6] << "," << estimator.X[7] << std::endl;
  controls_file << roll_cmd << "," << pitch_cmd << std::endl;
  k += 1; // sampling step
}

Eigen::MatrixXf sliding_window(Eigen::VectorXf y){
    // Measurement noise sliding window covariance matrix
    SW.row(T-1) = y;
    std::cout << "Matrix :\n" << SW.transpose() << std::endl;
    if (k<T){
        Eigen::MatrixXf centered = SW.topRows(k).rowwise() - SW.topRows(k).colwise().mean();
        Rn = (centered.adjoint() * centered) / double(SW.rows() - 1);
    }
    else{
        Eigen::MatrixXf centered = SW.rowwise() - SW.colwise().mean();
        Rn = (centered.adjoint() * centered) / double(SW.rows() - 1);
    }
    std::cout << "\nMeasurement online covariance matrix (Rn):\n" << Rn << "\n" << std::endl;
    SW.topRows(T-1) = SW.bottomRows(T-1); 
    return Rn;
}

Eigen::VectorXf setTarget(float x, float y, float z, float psi){
  target_x = x;
  target_y = y;
  target_z = z;
  target_psi = psi;
  
  ref << target_x, target_y, target_z, target_psi
  
  return ref;
}

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr &msg){
  curr_pos = *msg;
  if (target_set_state == 1) {
    auto start = std::chrono::high_resolution_clock::now();
    lqg(); //pid_pos_form();
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cct_file << elapsed.count() << std::endl;
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
  // Lets do it after 4s
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

using namespace std;
// Kalman filter constructor:
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
void KalmanFilter::correct ( Eigen::VectorXf Y) {
  K = ( P * C.transpose() ) * ( C * P * C.transpose() + R).inverse();
  X = X + K*(Y - C * X);
  P = (I - K * C) * P;
  X0 = X;
  P0 = P;
}

// MCC Kalman filter constructor:
MCCKalmanFilter::MCCKalmanFilter(int _n,  int _m, int _sigma) {
	n = _n;
	m = _m;
  sigma = _sigma;
}
// Set Fixed Matrix
void MCCKalmanFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}
// Set Fixed Matrix
void MCCKalmanFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}
// Set Initial Matrix
void MCCKalmanFilter::setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}
// Do prediction based of physical system (No external input)	 
void MCCKalmanFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}
// Do prediction based of physical system (with external input) U: Control vector	 
void MCCKalmanFilter::predict( Eigen::VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}
// Correct the prediction, using mesaurement  Y: mesaure vector
void MCCKalmanFilter::correct ( Eigen::VectorXf Y ) {
  // norm_innov = sqrt((innov).'*invers_R*(innov));
  // Gkernel = exp(-(pow(norm(Y - C*X),2)) /(2*pow(sigma,2)))/exp(-(pow(norm(X - A*X),2))/(2*pow(sigma,2)));
  Eigen::VectorXf innov(m);
  Eigen::VectorXf innovx(n);
  innov = Y-C*X;
  innovx = X-A*X;
  K = (P.inverse() + exp(-(pow(innov.norm(),2)) /(2*pow(sigma,2)))/exp(-(pow(innovx.norm(),2))/(2*pow(sigma,2))) * C.transpose()*R.inverse()*C).inverse() * exp(-(pow(innov.norm(),2)) /(2*pow(sigma,2)))/exp(-(pow(innovx.norm(),2))/(2*pow(sigma,2))) * C.transpose()*R.inverse();  X = X + K *(Y - C*X);
  P = (I - K*C) * P * (I - K*C).transpose() + (K*R*K.transpose());
  X0 = X;
  P0 = P;
}

// H infinity filter constructor:
HinfFilter::HinfFilter(int _n,  int _m, int _theta) {
	n = _n;
	m = _m;
  theta = _theta;
}
// Set Fixed Matrix
void HinfFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}
// Set Fixed Matrix
void HinfFilter::setFixed( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}
// Set Initial Matrix 
void HinfFilter::setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}
// Do prediction based of physical system (No external input)	 
void HinfFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}
// Do prediction based of physical system (with external input)  U: Control vector	 
void HinfFilter::predict( Eigen::VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}
// Correct the prediction, using mesaurement  Y: mesaure vector
void HinfFilter::correct ( Eigen::VectorXf Y ) {
  K = P * (I-theta*P+C.transpose()*R.inverse()*C*P).inverse()*C.transpose()*R.inverse();
  X = X + K * (Y - C*X);
  P = P * (I - theta*P + C.transpose() * R.inverse() * C * P).inverse();
  X0 = X;
  P0 = P;
}