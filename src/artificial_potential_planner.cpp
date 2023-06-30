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
#include "ros/package.h"
#include <fstream>

//const double Ts = 0.01;
const int obstacle_array_size = 7;


using namespace std;

struct obstacle{
	Eigen::Vector3d pos_obs;
	double _eta_obs_i;
	double radius;
};

class artificial_potential_planner {
	public:
		artificial_potential_planner();	

        void odom_callback( nav_msgs::Odometry odom );

		void init_obstacles();
		void stop_drone();

		void artificial_potential_planner_loop();	
		void run();
		Eigen::Vector3d attractive_force_calc();
		Eigen::Vector3d repulsive_force_calc( double x_obs , double y_obs, double z_obs ,double eta_obs_i , double radius_obstacle);
		

		void empty_txt();
		void log_data();

	private:
		ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;

		ros::Publisher _x_pub;
		ros::Publisher _y_pub;
		ros::Publisher _z_pub;

		ros::Publisher _x_dot_pub;
		ros::Publisher _y_dot_pub;
		ros::Publisher _z_dot_pub;

		ros::Publisher _x_dot_dot_pub;
		ros::Publisher _y_dot_dot_pub;
		ros::Publisher _z_dot_dot_pub;

		Eigen::VectorXd _x_ref;
		Eigen::VectorXd _y_ref;
		Eigen::VectorXd _z_ref;

		Eigen::VectorXd _x_ref_dot;
		Eigen::VectorXd _y_ref_dot;
		Eigen::VectorXd _z_ref_dot;

		Eigen::VectorXd _x_ref_dot_dot;
		Eigen::VectorXd _y_ref_dot_dot;
		Eigen::VectorXd _z_ref_dot_dot;

		Eigen::Vector3d _goal_pos;
		Eigen::Vector3d _pos;	
		Eigen::Vector3d _vel;	


		Eigen::Vector3d p;
		Eigen::Vector3d p_dot;	
		Eigen::Vector3d p_dot_dot_f;			

		obstacle obstacle_array[obstacle_array_size];

		int _rate;


		double _Ka , _Kb , _Kd , _K_obs , _eta_obs_i;
};




artificial_potential_planner::artificial_potential_planner(){
	_goal_pos[0] = 5;
	_goal_pos[1] = -7;
	_goal_pos[2] = -2;

	_Kb = 0.4;
	_Ka = _Kb*2;
	_Kd = 1;
	_K_obs = 0.04;
//	_eta_obs_i = 1.5;

	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &artificial_potential_planner::odom_callback, this);	


	_x_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/x_ref", 1);
	_y_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/y_ref", 1);
	_z_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/z_ref", 1);

	_x_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/x_dot_ref", 1);
	_y_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/y_dot_ref", 1);
	_z_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/z_dot_ref", 1);

	_x_dot_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/x_dot_dot_ref", 1);
	_y_dot_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/y_dot_dot_ref", 1);
	_z_dot_dot_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/z_dot_dot_ref", 1);

	init_obstacles();

	empty_txt();
}

void artificial_potential_planner::init_obstacles(){

	obstacle_array[0].pos_obs << 2,-2,2.5;
	obstacle_array[0]._eta_obs_i = 2;
	obstacle_array[0].radius = 0.5;

	obstacle_array[1].pos_obs << 6,-2,2.5;
	obstacle_array[1]._eta_obs_i = 2;
	obstacle_array[1].radius = 0.5;

	obstacle_array[2].pos_obs << 0,-6,2.5;
	obstacle_array[2]._eta_obs_i = 2;
	obstacle_array[2].radius = 0.5;

	obstacle_array[3].pos_obs << 4,-6,2.5;
	obstacle_array[3]._eta_obs_i = 2;
	obstacle_array[3].radius = 0.5;

	obstacle_array[4].pos_obs << 8,-6,2.5;
	obstacle_array[4]._eta_obs_i = 2;
	obstacle_array[4].radius = 0.5;

	obstacle_array[5].pos_obs << 2,-8,2.5;
	obstacle_array[5]._eta_obs_i = 2;
	obstacle_array[5].radius = 0.5;

	obstacle_array[6].pos_obs << 6,-8,2.5;
	obstacle_array[6]._eta_obs_i = 2;
	obstacle_array[6].radius = 0.5;
}


void artificial_potential_planner::odom_callback( nav_msgs::Odometry odom ) {

    Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    Eigen::Vector3d vel_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z); 
	
	Eigen::Matrix3d R_enu2ned;
	R_enu2ned << 1,0,0,0,-1,0,0,0,-1;

    _pos = R_enu2ned*pos_enu;		//trasformazione da enu a ned -> pb è in ned
	_vel = R_enu2ned*vel_enu;

}


Eigen::Vector3d artificial_potential_planner::attractive_force_calc(){

	Eigen::Vector3d e , f_attr;

	e = _goal_pos - _pos;
	if (e.norm() < 1){
		f_attr = _Ka*e; // -_Kd*_vel;
		_rate = 100;		
	}
	else{
		f_attr = _Kb*e/e.norm();
		_rate = 100;
	}


	return f_attr;
}

Eigen::Vector3d artificial_potential_planner::repulsive_force_calc( double x_obs , double y_obs, double z_obs, double eta_obs_i, double radius_obstacle){

	Eigen::Vector3d e , f_rep;

//	double eta_obs_i = radius_obstacle*2;
//	double dist_from_center = sqrt( (_pos[0]-x_obs)*(_pos[0]-x_obs) + (_pos[1]-y_obs)*(_pos[1]-y_obs) + (_pos[2]-z_obs)*(_pos[2]-z_obs) );
	double dist_from_center = sqrt( (_pos[0]-x_obs)*(_pos[0]-x_obs) + (_pos[1]-y_obs)*(_pos[1]-y_obs) );


	double eta_i = dist_from_center - radius_obstacle;

	Eigen::Vector3d grad_eta_i;
	grad_eta_i(0) = (_pos[0] - x_obs)/dist_from_center;
	grad_eta_i(1) = (_pos[1] - y_obs)/dist_from_center;
//	grad_eta_i(2) = (z_obs-_pos[2])/dist_from_center;
	grad_eta_i(2) = 0;

//	cout << "grad_eta:" << endl << grad_eta_i << endl;

	int gamma = 2;

	if (eta_i <= eta_obs_i){
		f_rep = _K_obs/pow(eta_i,2)*pow( (1/eta_i) - 1/eta_obs_i ,  gamma-1 )*grad_eta_i;
	//	cout << "f_rep:" << endl << f_rep << endl;	
	}
	else{
		f_rep.setZero();;
	}

	


	return f_rep;
}


void artificial_potential_planner::stop_drone(){
	std_msgs::Float64 msg;

	msg.data = _goal_pos(0);
	_x_pub.publish(msg);
	msg.data = _goal_pos(1);
	_y_pub.publish(msg);
	msg.data = _goal_pos(2);
	_z_pub.publish(msg);
	msg.data = 0;
	_x_dot_pub.publish(msg);
	_y_dot_pub.publish(msg);
	_z_dot_pub.publish(msg);
	_x_dot_dot_pub.publish(msg);
	_y_dot_dot_pub.publish(msg);
	_z_dot_dot_pub.publish(msg);
}



void artificial_potential_planner::artificial_potential_planner_loop(){

//	Eigen::Vector3d p , p_dot , p_dot_dot , p_dot_dot_f;
	Eigen::Vector3d  p_dot_dot;
	Eigen::Vector3d p_dot_old, p_dot_dot_old , p_dot_dot_old_f;
	p.setZero();
	p(2) = -1;
	p_dot.setZero();
	p_dot_dot.setZero();
	p_dot_old.setZero();
	p_dot_dot_old.setZero();
	p_dot_dot_old_f.setZero();
	std_msgs::Float64 msg;

	Eigen::Vector3d attractive_force , total_repulsive_force;
	bool finish = false;

	while(ros::ok){
		std::cout << "Insert x reference" << endl;
		std::cin >> _goal_pos[0];
		std::cout << "Insert y reference" << endl;
		std::cin >> _goal_pos[1];
		std::cout << "Insert z reference" << endl;
		std::cin >> _goal_pos[2];
		p = _pos;
		finish = false;
		while ( !finish ){
			attractive_force = attractive_force_calc();
			total_repulsive_force.setZero();
			for (int i = 0 ; i < obstacle_array_size ; i++){
				total_repulsive_force = total_repulsive_force + repulsive_force_calc( obstacle_array[i].pos_obs(0) , obstacle_array[i].pos_obs(1) , obstacle_array[i].pos_obs(2) , obstacle_array[i]._eta_obs_i , obstacle_array[i].radius);
			}

			p_dot = attractive_force + total_repulsive_force;

			cout << "f attr: " << endl <<attractive_force << endl;
			cout << "f rep: " << endl <<total_repulsive_force << endl;


			
			p = p + p_dot/_rate;		

			p_dot_dot = (p_dot - p_dot_old)*_rate;
			p_dot_dot_f = 0.9048*p_dot_dot_old_f + p_dot_dot_old*0.009516;  	//frequenza di taglio a 10 hz
			p_dot_dot_old_f = p_dot_dot_f;
			p_dot_dot_old = p_dot_dot;	
			p_dot_old = p_dot;	




			msg.data = p(0);
			_x_pub.publish(msg);
			msg.data = p(1);
			_y_pub.publish(msg);
			msg.data = p(2);
			_z_pub.publish(msg);

			msg.data = p_dot(0);
			_x_dot_pub.publish(msg);
			msg.data = p_dot(1);
			_y_dot_pub.publish(msg);
			msg.data = p_dot(2);
			_z_dot_pub.publish(msg);
			
			msg.data = p_dot_dot_f(0);
			_x_dot_dot_pub.publish(msg);
			msg.data = p_dot_dot_f(1);
			_y_dot_dot_pub.publish(msg);
			msg.data = p_dot_dot_f(2);
			_z_dot_dot_pub.publish(msg);

	//		cout << p_dot_dot << endl << endl;
	//		cout << "p:" << p << endl << endl;

			log_data();

			if ( (abs((_goal_pos[0]-_pos[0])) < 0.01) && (abs((_goal_pos[1]-_pos[1]))< 0.01) && (abs((_goal_pos[2]-_pos[2])) < 0.01)){
				stop_drone();
				finish = true;
			}


			usleep(1000000/_rate);
		}
	}
}


void artificial_potential_planner::empty_txt(){

    // "svuota" i file di testo

	std::string pkg_loc = ros::package::getPath("fsr_pkg");
	
    ofstream file_app_x(pkg_loc + "/data/app_x.txt");
    file_app_x << "";
    file_app_x.close();
    ofstream file_app_y(pkg_loc + "/data/app_y.txt");
    file_app_y << "";
    file_app_y.close();
    ofstream file_app_z(pkg_loc + "/data/app_z.txt");
    file_app_z << "";
    file_app_z.close();

    ofstream file_app_x_dot(pkg_loc + "/data/app_x_dot.txt");
    file_app_x_dot << "";
    file_app_x_dot.close();
    ofstream file_app_y_dot(pkg_loc + "/data/app_y_dot.txt");
    file_app_y_dot << "";
    file_app_y_dot.close();
    ofstream file_app_z_dot(pkg_loc + "/data/app_z_dot.txt");
    file_app_z_dot << "";
    file_app_z_dot.close();

    ofstream file_app_x_dot_dot(pkg_loc + "/data/app_x_dot_dot.txt");
    file_app_x_dot_dot << "";
    file_app_x_dot_dot.close();
    ofstream file_app_y_dot_dot(pkg_loc + "/data/app_y_dot_dot.txt");
    file_app_y_dot_dot << "";
    file_app_y_dot_dot.close();
    ofstream file_app_z_dot_dot(pkg_loc + "/data/app_z_dot_dot.txt");
    file_app_z_dot_dot << "";
    file_app_z_dot_dot.close();

}



void artificial_potential_planner::log_data(){
	cout << "logging" << endl;
	std::string pkg_loc = ros::package::getPath("fsr_pkg");

   ofstream file_app_x(pkg_loc + "/data/app_x.txt",std::ios_base::app);
    file_app_x << p(0) << endl;
    file_app_x.close();
    ofstream file_app_y(pkg_loc + "/data/app_y.txt",std::ios_base::app);
    file_app_y <<  p(1) << endl;
    file_app_y.close();
    ofstream file_app_z(pkg_loc + "/data/app_z.txt",std::ios_base::app);
    file_app_z <<  p(2) << endl;
    file_app_z.close();

    ofstream file_app_x_dot(pkg_loc + "/data/app_x_dot.txt",std::ios_base::app);
    file_app_x_dot << p_dot(0) << endl;
    file_app_x_dot.close();
    ofstream file_app_y_dot(pkg_loc + "/data/app_y_dot.txt",std::ios_base::app);
    file_app_y_dot << p_dot(1) << endl;
    file_app_y_dot.close();
    ofstream file_app_z_dot(pkg_loc + "/data/app_z_dot.txt",std::ios_base::app);
    file_app_z_dot << p_dot(2) << endl;
    file_app_z_dot.close();

    ofstream file_app_x_dot_dot(pkg_loc + "/data/app_x_dot_dot.txt",std::ios_base::app);
    file_app_x_dot_dot << p_dot_dot_f(0) << endl;
    file_app_x_dot_dot.close();
    ofstream file_app_y_dot_dot(pkg_loc + "/data/app_y_dot_dot.txt",std::ios_base::app);
    file_app_y_dot_dot << p_dot_dot_f(1) << endl;
    file_app_y_dot_dot.close();
    ofstream file_app_z_dot_dot(pkg_loc + "/data/app_z_dot_dot.txt",std::ios_base::app);
    file_app_z_dot_dot << p_dot_dot_f(2) << endl;
    file_app_z_dot_dot.close();
}





void artificial_potential_planner::run() {
	boost::thread artificial_potential_planner_loop_t ( &artificial_potential_planner::artificial_potential_planner_loop, this);

	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "artificial_potential_planner");
	artificial_potential_planner app;
	app.run();
	return 0;
}
