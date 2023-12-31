#include "ros/ros.h"
#include "Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include "boost/thread.hpp"
#include <gazebo_msgs/SpawnModel.h>
#include "ros/package.h"
#include <fstream>

using namespace std;

const int obstacle_array_size = 7; 	// number of obstacles


struct obstacle{
	Eigen::Vector3d pos_obs;		// center of the columns
	double _eta_obs_i;				// range of influnce
	double radius;					// radius of the columns
};

class rrt_planner {
	public:
		rrt_planner();	
		void rrt_planner_loop();	
		void run();

		void init_obstacles();
		bool collision_check_line(Eigen::Vector4d q1 , Eigen::Vector3d q2 );

		//for graphic representation
		void spawnNode( Eigen::Vector3d point , int node_number , int type);
		void spawnArrow(Eigen::Vector3d start_point, Eigen::Vector3d end_point , int arrow_number);
		void spawnSolnArrow(Eigen::Vector3d start_point, Eigen::Vector3d end_point , int arrow_number);


	private:
		ros::NodeHandle _nh;

		ros::Publisher _waypoint_pub;

		Eigen::Vector3d _goal_pos;
		Eigen::Vector3d _start_pos;
		
		Eigen::MatrixXd _roadmap;
		double _delta;
		double _iter_number;

		double _xmin, _xmax, _ymin, _ymax, _zmin, _zmax;	//bounds of the environment

		obstacle obstacle_array[obstacle_array_size];		//array of all obstacles
};




rrt_planner::rrt_planner(){
	_waypoint_pub = _nh.advertise<std_msgs::Float64MultiArray>("/high_level_planner/waypoints", 1);

	//parameters initialization

	_start_pos[0] = 0;
	_start_pos[1] = 0;
	_start_pos[2] = -1;

	_goal_pos[0] = 5;
	_goal_pos[1] = -7;
	_goal_pos[2] = -2;

	init_obstacles();

	_delta = 5;
	_iter_number = 10;

	_xmin = 0;
	_xmax = 10;
	_ymin = -10;
	_ymax = 0;
	_zmin = -5;
	_zmax = -0.5;
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
	double drone_collision_range = 0.215*2; //2 times the arm length to have a safety margin
	double x,y,z;
	Eigen::Vector3d direction;
	double distance = (q2.block<3,1>(0,0) - q1.block<3,1>(0,0)).norm();
	direction = (q2.block<3,1>(0,0) - q1.block<3,1>(0,0));
	direction.normalize();
	double step_size = 0.1;
	double step = step_size;

	while ( ( step < distance ) && !collision ){
		x = q1(0) + direction(0)*step;
		y = q1(1) + direction(1)*step;
		z = q1(2) + direction(2)*step;

		for (int i = 0 ; i<obstacle_array_size ; i++){
			if ( (pow(x-obstacle_array[i].pos_obs(0),2) + (pow(y-obstacle_array[i].pos_obs(1),2))) <  pow(obstacle_array[i].radius + drone_collision_range,2) )	{			//radius + arm length		
				std::cout << "collision" << endl;
				collision = true;
				return true;
			}
			if ( (q2(0)>_xmax) || (q2(0)<_xmin) || (q2(1)>_ymax)  || q2(1)<_ymin || q2(2)>_zmax || q2(2)<_zmin ){
				std::cout << "out of bound" << endl;
				collision = true;
				return true;				
			}
		}
		step = step + step_size;
	}
	return false;
}


void rrt_planner::rrt_planner_loop(){

	_roadmap.setZero();
	_roadmap.conservativeResize(4,1);		//initialize the roadmapa as 4 rows 1 column matrix  
	_roadmap.block<3,1>(0,0) = _start_pos;	//assign the start position to the roadmap
	_roadmap(3) = -1;						//to specify that the start position does not have a parent node

	bool finish;
	int iter = _iter_number;				//initialize the number of iterations

	double min_dist , dist;
	int min_index;

	Eigen::Vector3d q_rand , q_new;
	Eigen::Vector4d q_near;

	std_msgs::Float64MultiArray waypoints_msg;	//initialize the waypoint message
   	waypoints_msg.layout.dim.resize(1);	

	srand (time(NULL));		//extract random numbers
    	double rx , ry, rz; 


	while(ros::ok()){
		std::cout << "Insert x reference" << endl;
		std::cin >> _goal_pos[0];
		std::cout << "Insert y reference" << endl;
		std::cin >> _goal_pos[1];
		std::cout << "Insert z reference" << endl;
		std::cin >> _goal_pos[2];

		std::cout << "Trying with " << iter << " iterations" << endl;
		finish = false;
		iter = _iter_number;
		while(!finish){

			for( int i = 0 ; i < iter ; i++){		//extract q_rand from uniform distribution, within the bounds
				rx =  (rand() / (double)RAND_MAX) * (_xmax - _xmin) + _xmin;
				ry =  (rand() / (double)RAND_MAX) * (_ymax - _ymin) + _ymin;
				rz =  (rand() / (double)RAND_MAX) * (_zmax - _zmin) + _zmin;
				q_rand << rx,ry,rz ;


				min_dist = 1000000;		//find the node q_near with the minimum distance from q_rand
				min_index = -1;
				for(int j = 0 ; j <_roadmap.cols() ; j++){
					dist = ( _roadmap.block<3,1>(0,j)  - q_rand ).norm();
					if (dist < min_dist){
						min_dist = dist;
						min_index = j;
					}
				}
				q_near = _roadmap.block<4,1>(0,min_index);

				q_new = (q_near.block<3,1>(0,0) + (q_rand - q_near.block<3,1>(0,0))/min_dist*_delta  );	// q_new is at a distance delta from q_near

				if (collision_check_line(q_near,q_new) == false){		//if there is no collision in the line from q_near to q_new
					_roadmap.conservativeResize(4,_roadmap.cols()+1);		//add q_new to the roadmap
					_roadmap.block<3,1>(0,_roadmap.cols()-1) = q_new;
					_roadmap(3,_roadmap.cols()-1) = min_index;				//save q_near as parent of q_new
				}
			}

			min_dist = 1000000;		//find the node q_near with the minimum distance from goal
			min_index = -1;
			for(int j = 0 ; j <_roadmap.cols() ; j++){
				dist = ( _roadmap.block<3,1>(0,j)  - _goal_pos ).norm();
				std::cout << "waypoint number: "<< j << " dist: " << dist << endl;
				if (dist < min_dist){
					min_dist = dist;
					min_index = j;
				}
			}
			q_near = _roadmap.block<4,1>(0,min_index);

			if (collision_check_line(q_near,_goal_pos) == false){  //if there is no collision in the line from q_near to goal
				_roadmap.conservativeResize(4,_roadmap.cols()+1);	//add goal to the roadmap
				_roadmap.block<3,1>(0,_roadmap.cols()-1) = _goal_pos;
				_roadmap(3,_roadmap.cols()-1) = min_index;			//save q_near as parent of goal
				finish = true;
			}
			else{
				iter = iter + 2;
				std::cout << "Failure, now trying with "  << iter << "iterations" << endl;
			}	
		}

		std::cout << "Success" << endl;

		// graphics: spawn the graphical representation of the roadmap with nodes and arrows
		Eigen::Vector4d node;
		int parent_id;
		for (int i = 1 ; i < _roadmap.cols() ; i++){
			node = _roadmap.block<4,1>(0,i);
			parent_id = _roadmap(3,i);	
			if ( i != _roadmap.cols()-1 ){
				spawnNode( node.block<3,1>(0,0), i , 0);		//does not spawn goal
			}
			if (parent_id != -1){
				spawnArrow( node.block<3,1>(0,0) , _roadmap.block<3,1>(0,parent_id) , i);
			}
		}
		spawnNode(_goal_pos , 0 , 1);

		

		// build a matrix that contains all the points belonging to the path from start to goal
		Eigen::MatrixXd soln(3,1);
		soln.block<3,1>(0,0) = _goal_pos; 			//starts from goal node and goes backward until the start

		int parent = min_index;			//initialize parent node as the parent of the goal
		int k = 1;	
		do{
			soln.conservativeResize(3,k+1);
			soln.block<3,1>(0,k) = _roadmap.block<3,1>(0,parent);		
			parent = _roadmap(3,parent);
			k++;
		}while (parent != -1);			//fill the soln matrix from goal to start



		//graphics: spawn the arrows of the solution path
		for (int i = 0 ; i < soln.cols()-1 ; i++){
			spawnSolnArrow(  soln.block<3,1>(0,i) , soln.block<3,1>(0,i+1) , i);
		}


		//publish the obtained solution as a list of waypoints, towards the low level planner
		waypoints_msg.layout.dim[0].size = soln.cols()-1; 
		for (int i=k-2 ; i>=0 ; i--){
			std::cout << "node " << k-1-i << " : " << soln.block<3,1>(0,i) << endl;
			waypoints_msg.data.push_back(soln(0,i));
			waypoints_msg.data.push_back(soln(1,i));
			waypoints_msg.data.push_back(soln(2,i));
		}
		_waypoint_pub.publish(waypoints_msg);
	}
}



//graphics: calls a gazebo service to spawn node in the selected position
void rrt_planner::spawnNode( Eigen::Vector3d point , int node_number, int type){			//type 0->node type 1->goal
	ros::service::waitForService("/gazebo/spawn_urdf_model");
	ros::ServiceClient spawn_model_client = _nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	std::string pkg_loc = ros::package::getPath("fsr_pkg");

	 gazebo_msgs::SpawnModel spawn_model;
	std::stringstream model_buffer;	
	if (type == 0){
		std::ifstream model_file(pkg_loc + "/urdf/node.urdf");  
		model_buffer << model_file.rdbuf();
		spawn_model.request.model_xml = model_buffer.str();
		spawn_model.request.model_name = "node" + std::to_string(node_number);
		spawn_model.request.reference_frame = "world";
	}
	if (type == 1){
		std::ifstream model_file(pkg_loc + "/urdf/goal.urdf");  
		model_buffer << model_file.rdbuf();
		spawn_model.request.model_xml = model_buffer.str();
		spawn_model.request.model_name = "goal";
		spawn_model.request.reference_frame = "world";		
	}
	spawn_model.request.initial_pose.position.x = point[0];
	spawn_model.request.initial_pose.position.y = -point[1];
	spawn_model.request.initial_pose.position.z = -point[2];

	if (spawn_model_client.call(spawn_model))
		{
		//     ROS_INFO("Node spawned successfully!");
		}
	else
	    {
	        ROS_ERROR("Failed to spawn node.");
	    }
}

Eigen::Matrix3d skew(Eigen::Vector3d v){
	Eigen::Matrix3d skew;
	skew <<    0 , -v(2) , v(1),
		 v(2) , 0   , -v(0),
		-v(1) , v(0) ,    0;
	return skew;
}



//graphics: calls a gazebo service to spawn arrow in the selected position and orientation
void rrt_planner::spawnArrow(Eigen::Vector3d start_point, Eigen::Vector3d end_point , int arrow_number){
	ros::NodeHandle nh;
	ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	
	std::string pkg_loc = ros::package::getPath("fsr_pkg");
	
	std::ifstream model_file(pkg_loc + "/urdf/arrow.urdf");  
	std::stringstream model_buffer;
	model_buffer << model_file.rdbuf();
	std::string model_xml = model_buffer.str();
	
	start_point[1] = -start_point[1];
	start_point[2] = -start_point[2];
	end_point[1] = -end_point[1];
	end_point[2] = -end_point[2];

	// Calculate the length and orientation of the arrow
	double length = std::sqrt(std::pow(end_point[0] - start_point[0], 2) +
			      std::pow(end_point[1] - start_point[1], 2) +
			      std::pow(end_point[2] - start_point[2], 2));
	std::string length_placeholder = "0.777";  // Placeholder length in the URDF model
	std::string length_string = "" + std::to_string(length) ;
	size_t pos = model_xml.find(length_placeholder);
	if (pos != std::string::npos){
	model_xml.replace(pos, length_placeholder.length(), length_string);
	}
	
	length_placeholder = "0.555";  // Placeholder length in the URDF model
	length_string = "" + std::to_string(length/2);
	pos = model_xml.find(length_placeholder);
	if (pos != std::string::npos){
	model_xml.replace(pos, length_placeholder.length(), length_string);
	}

	// Prepare the SpawnModel service request
	gazebo_msgs::SpawnModel spawn_model;
	spawn_model.request.model_xml = model_xml;
	spawn_model.request.model_name = "arrow"  + std::to_string(arrow_number);
	spawn_model.request.reference_frame = "world";
	spawn_model.request.initial_pose.position.x = start_point[0];
	spawn_model.request.initial_pose.position.y = start_point[1];
	spawn_model.request.initial_pose.position.z = start_point[2];

    // Calculate orientation based on start and end points

	Eigen::Vector3d x(1,0,0);
	Eigen::Vector3d z = end_point - start_point;
	z.normalize();
	Eigen::Vector3d y = skew(z)*x;
	y.normalize();
	Eigen::Matrix3d R;
	R.block<3,1>(0,0) = skew(y)*z; 
	R.block<3,1>(0,1) = y;
	R.block<3,1>(0,2) = z;

	Eigen::Quaterniond quat(R);
	
	spawn_model.request.initial_pose.orientation.x = quat.x();
	spawn_model.request.initial_pose.orientation.y = quat.y();
	spawn_model.request.initial_pose.orientation.z = quat.z();
	spawn_model.request.initial_pose.orientation.w = quat.w();
	


	// Call the SpawnModel service
	if (spawn_model_client.call(spawn_model))
	{
	ROS_INFO("Arrow spawned successfully!");
	}
	else
	{
	ROS_ERROR("Failed to spawn arrow.");
	}
}


void rrt_planner::spawnSolnArrow(Eigen::Vector3d start_point, Eigen::Vector3d end_point , int arrow_number){
	ros::NodeHandle nh;
	ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	
	std::string pkg_loc = ros::package::getPath("fsr_pkg");
	
	std::ifstream model_file(pkg_loc + "/urdf/soln_arrow.urdf");  
	std::stringstream model_buffer;
	model_buffer << model_file.rdbuf();
	std::string model_xml = model_buffer.str();
	
	start_point[1] = -start_point[1];
	start_point[2] = -start_point[2];
	end_point[1] = -end_point[1];
	end_point[2] = -end_point[2];
	
	// Calculate the length and orientation of the arrow
	double length = std::sqrt(std::pow(end_point[0] - start_point[0], 2) +
			      std::pow(end_point[1] - start_point[1], 2) +
			      std::pow(end_point[2] - start_point[2], 2));
	std::string length_placeholder = "0.777";  // Placeholder length in the URDF model
	std::string length_string = "" + std::to_string(length) ;
	size_t pos = model_xml.find(length_placeholder);
	if (pos != std::string::npos){
		model_xml.replace(pos, length_placeholder.length(), length_string);
	}
	
	length_placeholder = "0.555";  // Placeholder length in the URDF model
	length_string = "" + std::to_string(length/2);
	pos = model_xml.find(length_placeholder);
	if (pos != std::string::npos){
		model_xml.replace(pos, length_placeholder.length(), length_string);
	}
	
	// Prepare the SpawnModel service request
	gazebo_msgs::SpawnModel spawn_model;
	spawn_model.request.model_xml = model_xml;
	spawn_model.request.model_name = "soln_arrow"  + std::to_string(arrow_number);
	spawn_model.request.reference_frame = "world";
	spawn_model.request.initial_pose.position.x = start_point[0];
	spawn_model.request.initial_pose.position.y = start_point[1];
	spawn_model.request.initial_pose.position.z = start_point[2];
	
	// Calculate orientation based on start and end points

	Eigen::Vector3d x(1,0,0);
	Eigen::Vector3d z = end_point - start_point;
	z.normalize();
	Eigen::Vector3d y = skew(z)*x;
	y.normalize();
	Eigen::Matrix3d R;
	R.block<3,1>(0,0) = skew(y)*z; 
	R.block<3,1>(0,1) = y;
	R.block<3,1>(0,2) = z;

	Eigen::Quaterniond quat(R);
	
	spawn_model.request.initial_pose.orientation.x = quat.x();
	spawn_model.request.initial_pose.orientation.y = quat.y();
	spawn_model.request.initial_pose.orientation.z = quat.z();
	spawn_model.request.initial_pose.orientation.w = quat.w();
	

	
	// Call the SpawnModel service
	if (spawn_model_client.call(spawn_model))
	{
	ROS_INFO("Arrow spawned successfully!");
	}
	else
	{
	ROS_ERROR("Failed to spawn arrow.");
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
