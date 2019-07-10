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

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <assert.h>
#include <complex>
#include <string>
#include <iostream>
#include <fstream>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define POS_THRESH 0.3

sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Imu curr_imu;
geometry_msgs::PointStamped local_position;
geometry_msgs::Point initial_pos;
geometry_msgs::PointStamped uwb_pos;
geometry_msgs::Point prev_pos;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 rpy;
dji_sdk_demo::Pos curr_pos;

bool set_local_position();

int target_set_state = 0;

int k = 0; // Sampling step
int n = 8; // Number of states
int m = 4; // Number of measurements
int j = 2; // Number of inputs
int r = 4; // Number of reference signals
int T = 2; // Sliding window size (samples)
int sigma = 100; // kernel size mcckf
float target_x;
float target_y;
float target_z;
float target_psi;
float x_error=0;
float y_error=0;
// control inputs
float roll_cmd = 0;
float pitch_cmd = 0;
float yaw_cmd = 0;
float z_cmd = 0;

double dt,dt1;
ros::Time start_, end_,s1,e1;

Eigen::MatrixXf A(n, n); // System dynamics matrix
Eigen::MatrixXf B(n, j); // Control input matrix
Eigen::MatrixXf C(m, n); // Output matrix
Eigen::MatrixXf Cc(j, n); // Control output matrix
//Eigen::MatrixXd Q(n, n); // State penalize matrix
//Eigen::MatrixXd R(j, j); // Input penalize matrix
Eigen::MatrixXf Qn(n, n); // Process noise covariance
Eigen::MatrixXf Rn(m, m); // Measurement noise covariance
Eigen::MatrixXf P0(n, n); // Estimate error covariance
Eigen::MatrixXf SW(m,T); // Sliding window measurement noise error covariance
Eigen::MatrixXf In(n, n); // Identity matrix
Eigen::MatrixXf Im(m, m); // Identity matrix
Eigen::MatrixXf L(j, n); // LQR control gain matrix
Eigen::MatrixXf Lc(j, j); // Gain filter tracking matrix
Eigen::VectorXf x0(n); // Initial states
Eigen::VectorXf u(j); // Control input vector
Eigen::VectorXf servo_compensator(j); // Lc(G)*ref
Eigen::VectorXf y(m); // Measurement vector
Eigen::VectorXf ref(j); // Target reference vector

void lqg();
Eigen::MatrixXf sliding_window(Eigen::VectorXf y, int i);
void setTarget(float x, float y, float z, float psi);

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

class KalmanFilter {
	public:
		/* Problem Dimension */
		int n; //State vector dimension
		int m; //Control vector (input) dimension (if there is not input, set to zero)
		/* Fixed Matrix */
		Eigen::MatrixXf A; //System dynamics matrix
		Eigen::MatrixXf B; //Control matrix 
		Eigen::MatrixXf C; //Mesaurement Adaptation matrix
		Eigen::MatrixXf Q; //Process Noise Covariance matrix
		Eigen::MatrixXf R; //Measurement Noise Covariance matrix
		Eigen::MatrixXf I; //Identity matrix
		/* Variable Matrix */
		Eigen::VectorXf X; //(Current) State vector
		Eigen::MatrixXf P; //State Covariance
		Eigen::MatrixXf K; //Kalman Gain matrix
		/* Inizial Value */
		Eigen::VectorXf X0; //Initial State vector
		Eigen::MatrixXf P0; //Initial State Covariance matrix
		/* 
		* Constructor 
		* _n: state vector dimension
		* _m: control vector dimension (if there is not input, set to zero)
		*/
		KalmanFilter(int _n,  int _m);
		/* Set Fixed Matrix (NO INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Fixed Matrix (WITH INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Initial Value */
		void setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 );
		/* Do prediction (NO INPUT) */
		void predict ( void );
		/* Do prediction (INPUT) */
		void predict ( Eigen::VectorXf U );
		/* Do correction */
		void correct ( Eigen::VectorXf Y); // add Eigen::MatrixXf _R for dynamic R matrix
};

class MCCKalmanFilter {
	public:
		/* Problem Dimension */
		int n; //State vector dimension
		int m; //Control vector (input) dimension (if there is not input, set to zero)
		int sigma;
		/* Fixed Matrix */
		Eigen::MatrixXf A; //System dynamics matrix
		Eigen::MatrixXf B; //Control matrix 
		Eigen::MatrixXf C; //Mesaurement Adaptation matrix
		Eigen::MatrixXf Q; //Process Noise Covariance matrix
		Eigen::MatrixXf R; //Measurement Noise Covariance matrix
		Eigen::MatrixXf I; //Identity matrix
		/* Variable Matrix */
		Eigen::VectorXf X; //(Current) State vector
		Eigen::MatrixXf P; //State Covariance
		Eigen::MatrixXf K; //Kalman Gain matrix
		/* Inizial Value */
		Eigen::VectorXf X0; //Initial State vector
		Eigen::MatrixXf P0; //Initial State Covariance matrix
		/* 
		* Constructor 
		* _n: state vector dimension
		* _m: control vector dimension (if there is not input, set to zero)
		*/
		MCCKalmanFilter(int _n,  int _m, int _sigma);
		/* Set Fixed Matrix (NO INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Fixed Matrix (WITH INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Initial Value */
		void setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 );
		/* Do prediction (NO INPUT) */
		void predict ( void );
		/* Do prediction (INPUT) */
		void predict ( Eigen::VectorXf U);
		/* Do correction */
		void correct ( Eigen::VectorXf Y );

};

class HinfFilter {
	public:
		/* Problem Dimension */
		int n; //State vector dimension
		int m; //Control vector (input) dimension (if there is not input, set to zero)
		int theta;
		/* Fixed Matrix */
		Eigen::MatrixXf A; //System dynamics matrix
		Eigen::MatrixXf B; //Control matrix 
		Eigen::MatrixXf C; //Mesaurement Adaptation matrix
		Eigen::MatrixXf Q; //Process Noise Covariance matrix
		Eigen::MatrixXf R; //Measurement Noise Covariance matrix
		Eigen::MatrixXf I; //Identity matrix
		/* Variable Matrix */
		Eigen::VectorXf X; //(Current) State vector
		Eigen::MatrixXf P; //State Covariance
		Eigen::MatrixXf K; //Kalman Gain matrix
		/* Inizial Value */
		Eigen::VectorXf X0; //Initial State vector
		Eigen::MatrixXf P0; //Initial State Covariance matrix
		/* 
		* Constructor 
		* _n: state vector dimension
		* _m: control vector dimension (if there is not input, set to zero)
		*/
		HinfFilter(int _n,  int _m, int _sigma);
		/* Set Fixed Matrix (NO INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Fixed Matrix (WITH INPUT) */
		void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _C, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
		/* Set Initial Value */
		void setInitial( Eigen::VectorXf _X0, Eigen::MatrixXf _P0 );
		/* Do prediction (NO INPUT) */
		void predict ( void );
		/* Do prediction (INPUT) */
		void predict ( Eigen::VectorXf U );
		/* Do correction */
		void correct ( Eigen::VectorXf Y );
};

