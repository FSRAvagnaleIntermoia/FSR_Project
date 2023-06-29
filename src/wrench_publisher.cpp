#include "ros/ros.h" 
#include "mav_msgs/Actuators.h"
#include "Eigen/Dense"

const int prop_number = 6;

using namespace std;

const double gravity = 9.81;
const double mass = 1.56779;
const double Ixx = 0.0347563;
const double Iyy = 0.0458929;
const double Izz = 0.0977;
const double C_T = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
const double C_Q = 0.016*0.000000854858;  // M_i = k_m * F_i
const double arm_length = 0.215;
const int motor_number = 6;


int main(int argc, char **argv) {

	ros::init(argc, argv,"wrench_publisher");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);

    mav_msgs::Actuators act_msg;
    act_msg.angular_velocities.resize(motor_number);

	Eigen::Matrix4Xd _allocation_matrix;
	_allocation_matrix.resize(4,motor_number);
	_allocation_matrix << C_T , C_T , C_T , C_T , C_T , C_T ,
						C_T*arm_length*sin(M_PI/6),  C_T*arm_length,  C_T*arm_length*sin(M_PI/6), -C_T*arm_length*sin(M_PI/6), -C_T*arm_length, -C_T*arm_length*sin(M_PI/6),
						C_T*arm_length*cos(M_PI/6),  0,  -C_T*arm_length*cos(M_PI/6),  -C_T*arm_length*cos(M_PI/6), 0, C_T*arm_length*cos(M_PI/6),
						C_Q,  -C_Q, C_Q, -C_Q, C_Q, -C_Q;
    
	Eigen::VectorXd angular_velocities_sq(motor_number);


	Eigen::VectorXd control_input(4);
	control_input(0) = mass*9.8;
	control_input(1) = 0.05;
	control_input(2) = 0;
	control_input(3) = 0;


	angular_velocities_sq =  _allocation_matrix.transpose()*(_allocation_matrix*_allocation_matrix.transpose()).inverse() * control_input;




	ros::Rate rate(10); 
	while ( ros::ok() ) {

		for (int i=0 ; i<motor_number ; i++){
			if (angular_velocities_sq(i) >= 0) 
		    act_msg.angular_velocities[i] = sqrt(angular_velocities_sq(i));	
			if (angular_velocities_sq(i) < 0) 
		    act_msg.angular_velocities[i] = -sqrt(-angular_velocities_sq(i));	
		}
		cout << angular_velocities_sq << endl << endl;


		cmd_pub.publish(act_msg);
		rate.sleep();		
	}
	
	return 0;
}