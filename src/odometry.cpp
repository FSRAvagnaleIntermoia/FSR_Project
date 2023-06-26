#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
//Include tf libraries							
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"


const double gravity = 9.8;
const double mass = 1.56779;

using namespace std;

class odometry {
	public:
		odometry();	

        void imu_callback( sensor_msgs::Imu imu  );
        void input_callback( std_msgs::Float64 u_T );


		void odometry_loop();	
		void run();
		
	private:

		Eigen::Vector3d _f_b;
		tf::Vector3 _omega_b_b;


		tf::Matrix3x3 _Rb;
		Eigen::Matrix3d _eigen_Rb;
	    tf::Matrix3x3 _R_enu2ned;
	    tf::Matrix3x3 _Q;

		ros::NodeHandle _nh;
        ros::Subscriber _imu_sub;
        ros::Subscriber _uT_sub;

        ros::Publisher _odom_pub;


		int _rate;
		bool _first_imu;
		bool _first_input;
};




odometry::odometry() : _R_enu2ned(1,0,0,0,-1,0,0,0,-1){
	_f_b.setZero();
	_Q.setIdentity();
	_Rb.setIdentity();	
	_first_imu = false;
	_first_input = false;
	_rate = 100;

	_odom_pub = _nh.advertise<nav_msgs::Odometry>("/firefly/odometry", 1);
	_uT_sub = _nh.subscribe("/firefly/uT", 0, &odometry::input_callback, this);	
	_imu_sub = _nh.subscribe("/firefly/imu", 0, &odometry::imu_callback, this);	
}

void odometry::input_callback( std_msgs::Float64 u_T ) {
	_f_b(2) = -u_T.data;
	_first_input = true;
}


void odometry::imu_callback( sensor_msgs::Imu imu ) {

	tf::Vector3 omega_wenu_b(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z); 

	tf::Vector3 omega_wned_b = _R_enu2ned.transpose()*omega_wenu_b;

	_omega_b_b = _Rb.transpose()*omega_wned_b;				//NED

	tf::Matrix3x3 R(tf::Quaternion(imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w));

    tf::Matrix3x3 R_ned2p = _R_enu2ned.transpose()*R*_R_enu2ned;

	_Rb = R_ned2p;		

	for(int i = 0 ; i < 3 ; i++){
		for (int j = 0 ; j <3 ; j++){
			_eigen_Rb(i,j) = _Rb[i][j];
		}
	}

	
	_first_imu = true;

}






void odometry::odometry_loop(){

	Eigen::Vector3d p , p_dot , p_dot_dot;// , p_dot_dot_f;
	Eigen::Vector3d e3(0,0,1);

	p.setZero();
	p(2) = -0.08;
	p_dot.setZero();
	p_dot_dot.setZero();
	nav_msgs::Odometry odom_msg;

	ros::Rate r(_rate);

	cout << "waiting first input "  << endl;
	while (!_first_imu || !_first_input) r.sleep();
	cout << "loop start"  << endl;
	while(ros::ok){


		p_dot_dot = (1/mass)*((mass*gravity*e3) + _eigen_Rb*_f_b);

		
		p_dot = p_dot + p_dot_dot/_rate;
		p = p + p_dot/_rate;		


		odom_msg.pose.pose.position.x = p(0);
		odom_msg.pose.pose.position.y = -p(1);
		odom_msg.pose.pose.position.z = -p(2);
		odom_msg.twist.twist.linear.x = p_dot(0);
		odom_msg.twist.twist.linear.y = -p_dot(1);
		odom_msg.twist.twist.linear.z = -p_dot(2);

		_odom_pub.publish(odom_msg);
		cout << p_dot_dot << endl << endl;
//		cout << "p:" << p << endl << endl;

		r.sleep();
	}
}


void odometry::run() {
	boost::thread odometry_loop_t ( &odometry::odometry_loop, this);
	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "odometry");
	odometry odo;
	odo.run();
	return 0;
}
