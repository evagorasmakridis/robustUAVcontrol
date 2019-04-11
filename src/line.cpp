#include "dji_sdk_demo/line.h"
#include "dji_sdk/dji_sdk.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include "KalmanFilter.h" 
#include "MCCKalmanFilter.h" 
#include "LinearQuadraticRegulator.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlRollPitchYawHeightPub;
ros::Publisher plotpos;
using namespace std::chrono;
high_resolution_clock::time_point t1;
high_resolution_clock::time_point t2;
high_resolution_clock::time_point t3;
high_resolution_clock::time_point t4;

float max_angle=0.09;

int main(int argc, char** argv)
{
  // t2 = high_resolution_clock::now();
  // t4 = high_resolution_clock::now();
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node

  int n = 12; // Number of states
  int m = 4; // Number of measurements
  int j = 4; // Number of inputs
  int T = 4; // Sliding window size
  VectorXf y(m); // Measurement vector
  VectorXf u(j); // Control input vector
  VectorXf ref(m); // Reference vector
  Eigen::MatrixXf A(n, n); // System dynamics matrix
  Eigen::MatrixXf B(n, j); // System dynamics matrix
  Eigen::MatrixXf C(m, n); // Output matrix
  Eigen::MatrixXf Qn(n, n); // Process noise covariance
  Eigen::MatrixXf Rn(m, m); // Measurement noise covariance
  Eigen::MatrixXf P0(n, n); // Estimate error covariance
  Eigen::MatrixXf In(n, n); // Identity matrix
  Eigen::MatrixXf Im(m, m); // Identity matrix
  Eigen::MatrixXf SW(T,m); // Estimate error covariance
  Eigen::MatrixXd Q(n, n); // State penalize matrix
  Eigen::MatrixXd R(j, j); // Input penalize matrix
  Eigen::MatrixXf K(j,n); // LQR gain matrix
  Eigen::VectorXf x0(n); // Initial states
  x0 << 0,0,0,0,0,0,0,0,0,0,0,0;
  int theta = 50000;
  int sigma = 1;
  
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

  // For Kalman filter
  In.setIdentity(n, n);
  Im.setIdentity(m,m);
  P0=10*In;
  Qn=5*In;
  Rn=5*Im;
  
  // For LQR
  Q.setIdentity(n, n);
  R.setIdentity(j, j);
  Q(7,7)=10;
  Q(9,9)=10;
  Q(11,11)=10;
  R(0,0)=0.001;

  /* Create and initialize the Kalman Filter*/
  KalmanFilter kf(n, m);
  kf.setFixed(A, C, Qn, Rn);
  kf.setInitial(x0, P0);

  /* Create and initialize the MCC Kalman Filter*/
  MCCKalmanFilter mcckf(n, m, sigma);
  mcckf.setFixed(A, C, Qn, Rn);
  mcckf.setInitial(x0, P0);

  /* Create and initialize the Hinf Filter*/
  HinfFilter hinf(n, m, theta);
  hinf.setFixed(A, C, Qn, Rn);
  hinf.setInitial(x0, P0);
  
  /* Create The LQR controller */
  LQR lqr(n,j);
  Eigen::MatrixXd Adbl = A.cast <double> (); // Matrix of floats to double.
  Eigen::MatrixXd Bdbl = B.cast <double> (); // Matrix of floats to double.
  lqr.setFixed(Adbl,Bdbl,Q,R);
  lqr.solver();
 
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  //::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  // ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  // ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  // ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber imu     =  nh.subscribe("/dji_sdk/imu",2,&imu_callback);
  
  //ros::Subscriber initialPosition = nh.subscribe("initial_pos", 1, &initial_pos_callback);

  // Publish the control signal
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;


   /*
  //imuAcc.open("distacc.txt");
  //	imuG.open("accels.txt");
  uwb.open("velpos30.txt");
  comm.open("velcomms30.txt");
  err.open("pid30.txt");
  err<<kp<<" "<<ki<<" "<<kd<<" "<<std::endl;
  */
  fileid="k1";
  open_data_files(fileid);

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
  ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
 
  // t2 = high_resolution_clock::now();
   //t4 = high_resolution_clock::now();
   if(takeoff_result){
      printf("READY TO CONTROL \n");
   }
   //ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
   //ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
 
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

void control(){
  if (k==0)
    u << 0,0,0,0;

  k += 1; // sampling

  // Measurement noise sliding window covariance matrix
  // SW.row(T-1) = y;
  // std::cout << "Matrix :\n" << SW.transpose() << std::endl;
  // if (k<T){
  //     MatrixXd centered = SW.topRows(k).rowwise() - SW.topRows(k).colwise().mean();
  //     R = (centered.adjoint() * centered) / double(SW.rows() - 1);
  // }
  // else{
  //     MatrixXd centered = SW.rowwise() - SW.colwise().mean();
  //     R = (centered.adjoint() * centered) / double(SW.rows() - 1);
  // }
  // std::cout << "Covariance matrix :\n" << R << std::endl;
  // SW.topRows(T-1) = SW.bottomRows(T-1);

  y << rpy.z, curr_pos.x, curr_pos.y, curr_pos.z;
  
  // Kalman Filter
  kf.predict(u); //Predict phase
  kf.correct(y); //Correction phase

  // MCC Kalman Filter
  mcckf.predict(u); //Predict phase
  mcckf.correct(y); //Correction phase

  // Hinf Filter
  hinf.predict(u); //Predict phase
  hinf.correct(y); //Correction phase
        
  //cout << "y" << k << ":\n" << y << endl;
  //cout << "x_est" << k << ":\n" << kf.X << endl;
  //cout << "Klqr" << k << ":\n" << lqr.Klqr << endl;

  // Reference tracking gain filter
  MatrixXf Klqr = lqr.Klqr.cast <float> (); // Matrix of doubles to floats.
  MatrixXf A_cl = (A-B*Klqr);
  MatrixXf G = -B.inverse()*A_cl*C.inverse();
  ref << 0,desired_pos.x,desired_pos.y,desired_pos.z;
  u << G*ref -(Klqr)*kf.X;

  // Bound the uav angles close to the linearized operating point
  if (fabs(roll) >= max_angle)
    u[1] = (roll>0) ? max_angle : -1 * max_angle;
  else
    u[1] = roll;

  if (fabs(pitch) >= max_angle)
    u[2] = (pitch>0) ? max_angle : -1 * max_angle;
  else
    u[2] = pitch;

  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(u[1]); // U2 - roll
  controlmsg.axes.push_back(u[2]); // U3 - pitch
  controlmsg.axes.push_back(u[0]); // U1 - height
  controlmsg.axes.push_back(u[3]); // U4 - yaw
  ctrlRollPitchYawHeightPub.publish(controlmsg);
}


void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr& msg){
  curr_pos = *msg;
  //ROS_INFO("I heard: [%d]",( posi.nses));
  control();
  int elapsed_time = ros::Time::now().toNSec();// - posi.nses;
  //-1 pitch opposite sign
  switch(target_set_state){
    case 0:
      desired_pos.x=3.5;
      desired_pos.y=2.5;
      desired_pos.z=1.0;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
          target_set_state=1;
        }
      break;
    case 1:
      desired_pos.x=7.5;
      desired_pos.y=2.5;
      desired_pos.z=1.0;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
          target_set_state=0;
        }     
      break;
  }

  write_data_flight();
  //comm<<_span.count()<<std::endl;
  //elapsed_time=ros::Time::now().toNSec()-elapsed_time+curr_pos.time; 
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  curr_imu=*msg;
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


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg){
  flight_status = msg->data;
}


void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  current_atti = msg->quaternion;
  rpy=toEulerAngle(current_atti);
  printf("Yaw is %f \n",rpy.z );
}

/*
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position=*msg;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

*/

bool takeoff_land(int task){
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result){
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control(){
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result){
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100(){
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31){
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
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10)){
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ){
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