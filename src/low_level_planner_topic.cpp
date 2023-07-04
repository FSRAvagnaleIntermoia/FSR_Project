#include "ros/ros.h"
#include "Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include "boost/thread.hpp"

const double Ts = 0.01;		//sampling time

using namespace std;

class low_level_planner {
	public:
		low_level_planner();	


		void low_level_planner_loop();	
		void waypoint_callback( std_msgs::Float64MultiArray msg );
		void run();

		void init_trajectory(double ttot , double x0, double y0, double z0, double psi0, double xf, double yf, double zf, double psif );
		void ref_var_filler( Eigen::VectorXd & var_ref, Eigen::VectorXd & var_ref_dot , Eigen::VectorXd & var_ref_dot_dot , Eigen::MatrixXf A , double var0 , double varf ,int idx);

	private:
		ros::NodeHandle _nh;

		ros::Publisher _ref_pub;

		ros::Subscriber _waypoint_sub;



		Eigen::VectorXd _x_ref;
		Eigen::VectorXd _y_ref;
		Eigen::VectorXd _z_ref;
		Eigen::VectorXd _psi_ref;

		Eigen::VectorXd _x_ref_dot;
		Eigen::VectorXd _y_ref_dot;
		Eigen::VectorXd _z_ref_dot;
		Eigen::VectorXd _psi_ref_dot;		

		Eigen::VectorXd _x_ref_dot_dot;
		Eigen::VectorXd _y_ref_dot_dot;
		Eigen::VectorXd _z_ref_dot_dot;
		Eigen::VectorXd _psi_ref_dot_dot;

		Eigen::MatrixXd _waypoints;		//this matrix contains as columns the points obtained by the high level planner
		int _n_waypoints;
		bool _first_waypoint;

		int _time_len;
		double _ttot, _x0, _y0, _z0, _psi0, _xf, _yf, _zf, _psif;
	
};


low_level_planner::low_level_planner(){

	_ref_pub = _nh.advertise<std_msgs::Float64MultiArray>("/low_level_planner/reference_trajectory", 1);

	_waypoint_sub = _nh.subscribe("/high_level_planner/waypoints", 0, &low_level_planner::waypoint_callback, this);	

	_x0 = 0;
	_y0 = 0;
	_z0 = -1;
	_psi0 = 0;
	_ttot = 20;

	_waypoints.resize(3,1);		//initialize the matrix as one column
	_waypoints.setZero();
	_first_waypoint = false;
}

void low_level_planner::waypoint_callback(std_msgs::Float64MultiArray msg){
	 _n_waypoints = msg.layout.dim[0].size;		//reads the total number of waypoints
	_waypoints.resize(3,_n_waypoints);			//resize to have the same number of columns as the number of waypoints
	for (int i = 0 ; i < _n_waypoints ; i++ ){	//fill the waypoints matrix
		_waypoints(0,i) = msg.data[3*i];
		_waypoints(1,i) = msg.data[3*i+1];
		_waypoints(2,i) = msg.data[3*i+2];
	}
	_first_waypoint  = true;
}

void low_level_planner::init_trajectory(double ttot , double x0, double y0, double z0, double psi0, double xf, double yf, double zf, double psif ){

	//this function uses 7th order polynomial to obtain reference values, we assume that initial and final velocity, acceleration and jerk are 0

	_time_len = ceil(ttot/Ts); //number of samples
	_x_ref.resize(_time_len);		//these vectors contain the reference at each time instant
	_y_ref.resize(_time_len);
	_z_ref.resize(_time_len);
	_psi_ref.resize(_time_len);
	_x_ref_dot.resize(_time_len);
	_y_ref_dot.resize(_time_len);
	_z_ref_dot.resize(_time_len);
	_psi_ref_dot.resize(_time_len);
	_x_ref_dot_dot.resize(_time_len);
	_y_ref_dot_dot.resize(_time_len);
	_z_ref_dot_dot.resize(_time_len);
	_psi_ref_dot_dot.resize(_time_len);

	Eigen::MatrixXf A(8,8);
	A << 0 , 0 , 0 , 0 , 0 , 0 , 0  , 1 ,
		pow(ttot,7), pow(ttot,6), pow(ttot,5), pow(ttot,4), pow(ttot,3), pow(ttot,2), ttot, 1,
		0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 ,
		7*pow(ttot,6), 6*pow(ttot,5), 5*pow(ttot,4), 4*pow(ttot,3) ,3*pow(ttot,2), 2*ttot, 1, 0,
		0 , 0 , 0 , 0 , 0 , 2 , 0 , 0 ,
		42*pow(ttot,5), 30*pow(ttot,4), 20*pow(ttot,3), 12*pow(ttot,2),6*ttot, 2, 0, 0,
		0 , 0 , 0 , 0 , 6 , 0 , 0 , 0 ,
		210*pow(ttot,4), 120*pow(ttot,3), 60*pow(ttot,2), 24*ttot ,6, 0, 0, 0;

	ref_var_filler(_x_ref,_x_ref_dot,_x_ref_dot_dot,A,x0,xf,0);
	ref_var_filler(_y_ref,_y_ref_dot,_y_ref_dot_dot,A,y0,yf,1);
	ref_var_filler(_z_ref,_z_ref_dot,_z_ref_dot_dot,A,z0,zf,2);
	ref_var_filler(_psi_ref,_psi_ref_dot,_psi_ref_dot_dot,A,psi0,psif,3);

}

void low_level_planner::ref_var_filler( Eigen::VectorXd & var_ref, Eigen::VectorXd & var_ref_dot , Eigen::VectorXd & var_ref_dot_dot , Eigen::MatrixXf A , double var0 , double varf ,int idx){

	//this function uses the coefficient given by the computed A matrix to construct the time law for each variable
	
	Eigen::MatrixXf b(8,1);
	b.setZero();		// the initial and final value of the derivatives are 0
	b(0) = var0;		// initial value of the variable
	b(1) = varf;		// final value of the variable
	Eigen::MatrixXf a_temp(8,1);
	Eigen::Vector4d a7 , a6, a5 , a4 , a3 , a2 , a1 , a0;
	a_temp = A.inverse()*b;
	a7(idx) = a_temp(0);
	a6(idx) = a_temp(1);
	a5(idx) = a_temp(2);
	a4(idx) = a_temp(3);
	a3(idx) = a_temp(4);
	a2(idx) = a_temp(5);
	a1(idx) = a_temp(6);
	a0(idx) = a_temp(7);

	double t = 0;
	for (int i = 0 ; i < _time_len ; i++){
		t = i*Ts;
		var_ref(i) = a7(idx)*pow(t,7) + a6(idx)*pow(t,6) + a5(idx)*pow(t,5) + a4(idx)*pow(t,4) + a3(idx)*pow(t,3) + a2(idx)*pow(t,2) +  a1(idx)*t + a0(idx);  
		var_ref_dot(i) = 7*a7(idx)*pow(t,6) + 6*a6(idx)*pow(t,5) + 5*a5(idx)*pow(t,4) + 4*a4(idx)*pow(t,3) + 3*a3(idx)*pow(t,2) + 2*a2(idx)*t +  a1(idx) ;  
		var_ref_dot_dot(i) = 42*a7(idx)*pow(t,5) + 30*a6(idx)*pow(t,4) + 20*a5(idx)*pow(t,3) + 12*a4(idx)*pow(t,2) + 6*a3(idx)*t + 2*a2(idx) ;  
	}		
}




void low_level_planner::low_level_planner_loop() {	
	ros::Rate rate(100);
	int time_idx = 0;
	std_msgs::Float64MultiArray msg;
	while(ros::ok){

	cout << "Waiting for reference" << endl;
	while (!_first_waypoint) rate.sleep();
	
	cout << "Waypoints acquired" << endl;
	cout << "Trajectory start" << endl;
		for ( int i = 0 ; i < _n_waypoints ; i++){
			_xf = _waypoints(0,i);
			_yf = _waypoints(1,i);
			_zf = _waypoints(2,i);
			_psif = 0;
			cout << "Waypoint number: " << i+1 << endl;
			cout << _xf << endl << _yf << endl << _zf << endl << endl;
			init_trajectory(_ttot,_x0,_y0,_z0,_psi0,_xf,_yf,_zf,_psif);

		while (time_idx < _time_len){		//publish the reference at each time instant

				msg.data = { _x_ref(time_idx) , _y_ref(time_idx) , _z_ref(time_idx) , _psi_ref(time_idx) , 
							_x_ref_dot(time_idx) , _y_ref_dot(time_idx) , _z_ref_dot(time_idx) , _psi_ref_dot(time_idx) , 
							_x_ref_dot_dot(time_idx) , _y_ref_dot_dot(time_idx) , _z_ref_dot_dot(time_idx) , _psi_ref_dot_dot(time_idx) };

				_ref_pub.publish(msg);

				time_idx ++;	
				rate.sleep();
			}


			_x0 = _xf;				//start a new trajectory from the last point
			_y0 = _yf;
			_z0 = _zf;
			_psi0 = _psif;

			time_idx = 0;	

		}
		_first_waypoint  = false;	
	}
}

void low_level_planner::run() {
	boost::thread low_level_planner_loop_t ( &low_level_planner::low_level_planner_loop, this);
	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "low_level_planner");
	low_level_planner llp;
	llp.run();
	return 0;
}
