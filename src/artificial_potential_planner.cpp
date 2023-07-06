#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include "boost/thread.hpp"
#include "ros/package.h"
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <fstream>

using namespace std;

const int obstacle_array_size = 7; 	// number of obstacles


struct obstacle{
	Eigen::Vector3d pos_obs;		// center of the columns
	double _eta_obs_i;				// range of influnce
	double radius;					// radius of the columns
};

class artificial_potential_planner {
	public:
		artificial_potential_planner();	
		void artificial_potential_planner_loop();	
		void run();

        void odom_callback( nav_msgs::Odometry odom );			

		void init_obstacles();
		void stop_drone();

		Eigen::Vector3d attractive_force_calc();
		Eigen::Vector3d repulsive_force_calc( double x_obs , double y_obs, double z_obs ,double eta_obs_i , double radius_obstacle);
		
		//for data logging		
		void empty_txt();
		void log_data();
		//for graphics
		void setModelPose(const Eigen::Vector3d& force);

	private:
		ros::NodeHandle _nh;
       		ros::Subscriber _odom_sub;

		ros::Publisher _ref_pub;
		ros::Publisher _gazebo_pub;	//for graphics


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


		Eigen::Vector3d p;
		Eigen::Vector3d p_dot;	
		Eigen::Vector3d p_dot_dot_f;			

		obstacle obstacle_array[obstacle_array_size]; //array of all obstacles
		
		double _Ka , _Kb , _K_obs , _eta_obs_i;
};




artificial_potential_planner::artificial_potential_planner(){
	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &artificial_potential_planner::odom_callback, this);	

	_ref_pub = _nh.advertise<std_msgs::Float64MultiArray>("/low_level_planner/reference_trajectory", 1);
    	_gazebo_pub = _nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

	//initialize parameters

	_goal_pos[0] = 5;
	_goal_pos[1] = -7;
	_goal_pos[2] = -2;

	_Kb = 0.4;
	_Ka = _Kb; 		//so that fa = fb when ||e|| = 1
	_K_obs = 0.04;

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
	// to read current position
	Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	
	Eigen::Matrix3d R_enu2ned;
	R_enu2ned << 1,0,0,0,-1,0,0,0,-1;

	_pos = R_enu2ned*pos_enu;		//transform from world-ENU in world-NED
}


Eigen::Vector3d artificial_potential_planner::attractive_force_calc(){

	Eigen::Vector3d e , f_attr;

	e = _goal_pos - _pos;
	
	if (e.norm() < 1){
		f_attr = _Ka*e;
	}
	else{
		f_attr = _Kb*e/e.norm();
	}

	return f_attr;
}

Eigen::Vector3d artificial_potential_planner::repulsive_force_calc( double x_obs , double y_obs, double z_obs, double eta_obs_i, double radius_obstacle){

	Eigen::Vector3d e , f_rep;

	double dist_from_center = sqrt( (_pos[0]-x_obs)*(_pos[0]-x_obs) + (_pos[1]-y_obs)*(_pos[1]-y_obs) );		//the z component is ignored because we supposed that the obstacles are columns

	double eta_i = dist_from_center - radius_obstacle;		//distance from the surface of the obstacles

	Eigen::Vector3d grad_eta_i;								//analytic computation of the gradient
	grad_eta_i(0) = (_pos[0] - x_obs)/dist_from_center;
	grad_eta_i(1) = (_pos[1] - y_obs)/dist_from_center;
	grad_eta_i(2) = 0;

	int gamma = 2;
	if (eta_i <= eta_obs_i){
		f_rep = _K_obs/pow(eta_i,2)*pow( (1/eta_i) - 1/eta_obs_i ,  gamma-1 )*grad_eta_i;
	}
	else{
		f_rep.setZero();;
	}

	return f_rep;
}

//set the position reference as the goal, and zero for velocity and acceleration 
void artificial_potential_planner::stop_drone(){
	std_msgs::Float64MultiArray msg;
	cout << "stop " << endl;
	msg.data = {_goal_pos(0) , _goal_pos(1) , _goal_pos(2) , 0 ,
				0 , 0 , 0 , 0 ,
				0 , 0 , 0 , 0 };	
	while(ros::ok){
		_ref_pub.publish(msg);
		log_data();
	}
}



void artificial_potential_planner::artificial_potential_planner_loop(){

	Eigen::Vector3d  p_dot_dot;
	Eigen::Vector3d p_dot_old, p_dot_dot_old , p_dot_dot_old_f;

	p.setZero();	// initialize position
	p(2) = -1;

	p_dot.setZero();
	p_dot_dot.setZero();
	p_dot_old.setZero();
	p_dot_dot_old.setZero();
	p_dot_dot_old_f.setZero();


	std_msgs::Float64MultiArray msg;

	Eigen::Vector3d attractive_force , total_repulsive_force;
	bool finish = false;		

	double rate = 100;
	ros::Rate r(100);

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
			attractive_force = attractive_force_calc();		//compute attractive force
			total_repulsive_force.setZero();
			for (int i = 0 ; i < obstacle_array_size ; i++){		//compute repulsive force for each obstacle
				total_repulsive_force = total_repulsive_force + repulsive_force_calc( obstacle_array[i].pos_obs(0) , obstacle_array[i].pos_obs(1) , obstacle_array[i].pos_obs(2) , obstacle_array[i]._eta_obs_i , obstacle_array[i].radius);
			}

			p_dot = attractive_force + total_repulsive_force;	//compute total force and use it as velocity reference

			p = p + p_dot/rate;		//obtain position reference by integration

			p_dot_dot = (p_dot - p_dot_old)*rate;		//obtain acceleration reference by numeric derivation , and filter to reduce noise
			p_dot_dot_f = 0.9048*p_dot_dot_old_f + p_dot_dot_old*0.009516;  	//cutoff frequency 10 hz
			p_dot_dot_old_f = p_dot_dot_f;
			p_dot_dot_old = p_dot_dot;	
			p_dot_old = p_dot;	

			//publish the computed position reference, maintain psi ref=0
			msg.data = {p(0) , p(1) , p(2) , 0 ,
						p_dot(0) , p_dot(1) , p_dot(2) , 0 ,
						p_dot_dot_f(0) ,  p_dot_dot_f(1) ,  p_dot_dot_f(2) , 0};
			_ref_pub.publish(msg);



		    setModelPose(p_dot);		//graphics: draw the total force arrow

			log_data();

			//call the stop if the drone is near the goal
			if ( (abs((_goal_pos[0]-_pos[0])) < 0.01) && (abs((_goal_pos[1]-_pos[1]))< 0.01) && (abs((_goal_pos[2]-_pos[2])) < 0.01)){
				stop_drone();
				finish = true;
			}

			r.sleep();
//			usleep(1000000/rate);
		}
	}
}

Eigen::Matrix3d skew(Eigen::Vector3d v){
	Eigen::Matrix3d skew;
	skew <<    0 , -v(2) , v(1),
			 v(2) , 0   , -v(0),
			-v(1) , v(0) ,    0;
	return skew;
}
//graphics: draw the total force arrow in real time
void artificial_potential_planner::setModelPose(const Eigen::Vector3d & force ) {
 	gazebo_msgs::ModelState stateMsg;
	stateMsg.model_name = "total_force_arrow";
	stateMsg.reference_frame = "world"; // Or any other reference frame you desire	
	geometry_msgs::Pose pose;
	if( force.norm() > 0.1){
		Eigen::Vector3d x(1,0,0);
		Eigen::Vector3d z;
		z(0) = force(0);
		z(1) = -force(1);
		z(2) = -force(2);
		z.normalize();
		Eigen::Vector3d y = skew(z)*x;
		y.normalize();
		Eigen::Matrix3d R;
		R.block<3,1>(0,0) = skew(y)*z; 
		R.block<3,1>(0,1) = y;
		R.block<3,1>(0,2) = z;

		Eigen::Quaterniond quat(R);

		pose.orientation.x = quat.x();
		pose.orientation.y = quat.y();
		pose.orientation.z = quat.z();
		pose.orientation.w = quat.w();


		pose.position.x = _pos(0);
		pose.position.y = -_pos(1);
		pose.position.z = -_pos(2);



		stateMsg.pose = pose;
		_gazebo_pub.publish(stateMsg);
	}
	else{
		pose.position.z = -10;
		stateMsg.pose = pose;
		_gazebo_pub.publish(stateMsg);

	}
}

//for data logging
void artificial_potential_planner::empty_txt(){
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


//for data logging
void artificial_potential_planner::log_data(){
	//	cout << "logging" << endl;
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
