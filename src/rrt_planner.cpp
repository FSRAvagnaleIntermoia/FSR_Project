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

//const double Ts = 0.01;
const int obstacle_array_size = 7;


using namespace std;

struct obstacle{
	Eigen::Vector3d pos_obs;
	double _eta_obs_i;
	double radius;
};

class rrt_planner {
	public:
		rrt_planner();	

		void init_obstacles();
		bool collision_check_line(Eigen::Vector4d q1 , Eigen::Vector3d q2 );

		void rrt_planner_loop();	
		void run();
		Eigen::Vector3d attractive_force_calc();
		Eigen::Vector3d repulsive_force_calc( double x_obs , double y_obs, double z_obs ,double eta_obs_i , double radius_obstacle);
		
	private:
		ros::NodeHandle _nh;

		ros::Publisher _x_pub;
		ros::Publisher _y_pub;
		ros::Publisher _z_pub;

		Eigen::Vector3d _goal_pos;
		Eigen::Vector3d _start_pos;
		
		Eigen::MatrixXd _roadmap;
		double _delta;
		double _iter_number;

		obstacle obstacle_array[obstacle_array_size];

		int rate;
};




rrt_planner::rrt_planner(){
	_start_pos[0] = 0;
	_start_pos[1] = 0;
	_start_pos[2] = -1;

	_goal_pos[0] = 5;
	_goal_pos[1] = -7;
	_goal_pos[2] = -2;

	_x_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/x_ref", 1);
	_y_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/y_ref", 1);
	_z_pub = _nh.advertise<std_msgs::Float64>("/firefly/planner/z_ref", 1);

	init_obstacles();

	_delta = 2;
	_iter_number = 20;
}

void rrt_planner::init_obstacles(){

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


bool rrt_planner::collision_check_line(Eigen::Vector4d q1 , Eigen::Vector3d q2 ){
	bool collision = false;
	double drone_collision_range = 0.215*2; //arm length*2
	double x,y,z;
	Eigen::Vector3d d;
	d = (q2.block<3,1>(0,0) - q1.block<3,1>(0,0))/(q2.block<3,1>(0,0) - q1.block<3,1>(0,0)).norm();
	double k = 0.1;
	while ( ( k < (q2.block<3,1>(0,0) - q1.block<3,1>(0,0)).norm() ) && !collision ){
		x = q1(0) + d(0)*k;
		y = q1(1) + d(1)*k;
		z = q1(2) + d(2)*k;

		cout << x << endl << y << endl << z << endl << endl;

		for (int i = 0 ; i<obstacle_array_size ; i++){
			if ( (pow(x-obstacle_array[i].pos_obs(0),2) + (pow(y-obstacle_array[i].pos_obs(1),2))) <  pow(obstacle_array[i].radius + drone_collision_range,2) )	{			//radius + arm length		
				cout << "collision" << endl;
				collision = true;
				return true;
			}
		}
		cout << "k: " << k << endl;
		k = k + 0.1;
	}
	return false;
}



void rrt_planner::rrt_planner_loop(){

	_roadmap.conservativeResize(4,1);
	_roadmap.block<3,1>(0,0) = _start_pos;
	_roadmap(3) = -1;

	bool finish = false;
	int iter = _iter_number;

    double rx , ry, rz; rx =  (rand() / (double)RAND_MAX) * (10 - 0) + 0;

	double min_dist , dist;
	int min_index;

	Eigen::Vector3d q_rand , q_new;
	Eigen::Vector4d q_near;
	srand (time(NULL));

	double xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = 0;
	xmax = 10;
	ymin = -10;
	ymax = 0;
	zmin = -5;
	zmax = -0.5;


	while(ros::ok){
		cout << "Trying with " << iter << " iterations" << endl;
		while(!finish){

			for( int i = 0 ; i < iter ; i++){
				rx =  (rand() / (double)RAND_MAX) * (xmax - xmin) + xmin;
				ry =  (rand() / (double)RAND_MAX) * (ymax - ymin) + ymin;
				rz =  (rand() / (double)RAND_MAX) * (zmax - zmin) + zmin;
				q_rand << rx,ry,rz ;
				cout << "q_rand: " << q_rand << endl;

				min_dist = 10000;
				min_index = -1;
				for(int j = 0 ; j <_roadmap.cols() ; j++){
					dist = ( _roadmap.block<3,1>(0,j)  - q_rand ).norm();
					if (dist < min_dist){
						min_dist = dist;
						min_index = j;
					}
				}
				cout << "min dist:" << min_dist << endl;				
				q_near = _roadmap.block<4,1>(0,min_index);
				q_new = (q_near.block<3,1>(0,0) + (q_rand - q_near.block<3,1>(0,0))/min_dist*_delta  );

				if (collision_check_line(q_near,q_new) == false){
					_roadmap.conservativeResize(4,_roadmap.cols()+1);
					_roadmap.block<3,1>(0,_roadmap.cols()-1) = q_new;
					_roadmap(3,_roadmap.cols()-1) = min_index;
				}
	//			usleep(1000000);
				cout << "q_near: " << q_near << endl;
				cout << "q_new: " << q_new << endl<<endl;
			}

			min_dist = 10000;
			min_index = -1;
			for(int j = 0 ; j <_roadmap.cols() ; j++){
				dist = ( _roadmap.block<3,1>(0,j)  - _goal_pos ).norm();
				if (dist < min_dist){
					dist = min_dist;
					min_index = j;
				}
			}
			q_near = _roadmap.block<4,1>(0,min_index);

			if (collision_check_line(q_near,_goal_pos) == false){
				_roadmap.conservativeResize(4,_roadmap.cols()+1);
				_roadmap.block<3,1>(0,_roadmap.cols()-1) = _goal_pos;
				_roadmap(3,_roadmap.cols()-1) = min_index;
				finish = true;
			}
			else{
				iter = iter + 2;
			    cout << "Failure, now trying with "  << iter << "iterations" << endl;
			}	
		}



			cout << "Success !!!" << endl;

			Eigen::MatrixXd soln(3,1);
			soln.block<3,1>(0,0) = _goal_pos; 
			int parent = min_index;
			int k = 1;
			do{
				soln.conservativeResize(3,k+1);
				soln.block<3,1>(0,k) = _roadmap.block<3,1>(0,parent);	
				parent = _roadmap(3,parent);
				k++;
			//	cout << "roadmap:" << _roadmap << endl << endl << endl;
				cout << "parent: " << parent << endl;
				usleep(1000000);	
			}while (parent != -1);


/*			for (int i=k ; i>=0 ; i--){
				cout << "nodo:" << i << " : " << soln.block<3,1>(0,i) << endl;
			}*/

			for (int i=0 ; i<k ; i++){
				cout << "nodo:" << i << " : " << soln.block<3,1>(0,i) << endl;
			}
	}
}


void rrt_planner::run() {
	boost::thread rrt_planner_loop_t ( &rrt_planner::rrt_planner_loop, this);
	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "rrt_planner");
	rrt_planner rtt;
	rtt.run();
}
