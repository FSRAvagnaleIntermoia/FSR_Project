#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Wrench.h"


//Include tf libraries							
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"


using namespace std;

const double gravity = 9.8;
const double mass = 1.56779;
const double Ixx = 0.0347563;
const double Iyy = 0.0458929;
const double Izz = 0.0977;
const double C_T = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
const double C_Q = 0.016*0.00000854858;  // M_i = k_m * F_i
const double arm_length = 0.215;
const int motor_number = 6;
const double Ts = 0.01;




class geometric_controller {
	public:
		geometric_controller();			
		void ctrl_loop();
        void run();




		void x_ref_callback( std_msgs::Float64 msg);
		void y_ref_callback( std_msgs::Float64 msg);
		void z_ref_callback( std_msgs::Float64 msg);
		void psi_ref_callback( std_msgs::Float64 msg);
		void x_dot_ref_callback( std_msgs::Float64 msg);
		void y_dot_ref_callback( std_msgs::Float64 msg);
		void z_dot_ref_callback( std_msgs::Float64 msg);
		void psi_dot_ref_callback( std_msgs::Float64 msg);
		void x_dot_dot_ref_callback( std_msgs::Float64 msg);
		void y_dot_dot_ref_callback( std_msgs::Float64 msg);
		void z_dot_dot_ref_callback( std_msgs::Float64 msg);
		void psi_dot_dot_ref_callback( std_msgs::Float64 msg);





        void odom_callback( nav_msgs::Odometry odom );
		void imu_callback( sensor_msgs::Imu imu);


	private:
		ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _imu_sub;

		ros::Subscriber _x_sub;
		ros::Subscriber _y_sub;
		ros::Subscriber _z_sub;
		ros::Subscriber _psi_sub;

		ros::Subscriber _x_dot_sub;
		ros::Subscriber _y_dot_sub;
		ros::Subscriber _z_dot_sub;
		ros::Subscriber _psi_dot_sub;

		ros::Subscriber _x_dot_dot_sub;
		ros::Subscriber _y_dot_dot_sub;
		ros::Subscriber _z_dot_dot_sub;
		ros::Subscriber _psi_dot_dot_sub;



        ros::Publisher _act_pub;
        ros::Publisher _wrench_pub;

		Eigen::Vector3d _pos_ref;
		Eigen::Vector3d	_pos_ref_dot;
		Eigen::Vector3d	_pos_ref_dot_dot;

		double _psi_ref;
		double _psi_ref_dot;
		double _psi_ref_dot_dot;

		Eigen::Matrix3d _Rb;
	    Eigen::Matrix3d _R_enu2ned;

		Eigen::Matrix3d _Ib;

		Eigen::Matrix4Xd _allocation_matrix;

		Eigen::Vector3d _omega_b_b;
		Eigen::Vector3d _p_b;
		Eigen::Vector3d _p_b_dot;


//		tf::Matrix3x3 _Kp;
//		tf::Matrix3x3 _Ke;
		Eigen::Matrix3d _KR;
		double _Kp;
		double _Kv;
//		double _Ke;
//		double _Ke_dot;
		double _Ki;
//		double _Ki_e;


		Eigen::Matrix3d _KW;
		Eigen::Matrix3d _Ki_R;



		double _u_T;

		Eigen::Vector3d _tau_b;

		bool _first_imu;
		bool _first_odom;
		bool _control_on;
};



geometric_controller::geometric_controller() : _pos_ref(0,0,-1) , _psi_ref(0) , _pos_ref_dot(0,0,0) , _psi_ref_dot(0), _pos_ref_dot_dot(0,0,0) , _psi_ref_dot_dot(0) {

	_x_sub = _nh.subscribe("/firefly/planner/x_ref", 0, &geometric_controller::x_ref_callback, this);	
	_y_sub = _nh.subscribe("/firefly/planner/y_ref", 0, &geometric_controller::y_ref_callback, this);	
	_z_sub = _nh.subscribe("/firefly/planner/z_ref", 0, &geometric_controller::z_ref_callback, this);	
	_psi_sub = _nh.subscribe("/firefly/planner/psi_ref", 0, &geometric_controller::psi_ref_callback, this);	
	_x_dot_sub = _nh.subscribe("/firefly/planner/x_dot_ref", 0, &geometric_controller::x_dot_ref_callback, this);	
	_y_dot_sub = _nh.subscribe("/firefly/planner/y_dot_ref", 0, &geometric_controller::y_dot_ref_callback, this);	
	_z_dot_sub = _nh.subscribe("/firefly/planner/z_dot_ref", 0, &geometric_controller::z_dot_ref_callback, this);	
	_psi_dot_sub = _nh.subscribe("/firefly/planner/psi_dot_ref", 0, &geometric_controller::psi_dot_ref_callback, this);	
	_x_dot_dot_sub = _nh.subscribe("/firefly/planner/x_dot_dot_ref", 0, &geometric_controller::x_dot_dot_ref_callback, this);	
	_y_dot_dot_sub = _nh.subscribe("/firefly/planner/y_dot_dot_ref", 0, &geometric_controller::y_dot_dot_ref_callback, this);	
	_z_dot_dot_sub = _nh.subscribe("/firefly/planner/z_dot_dot_ref", 0, &geometric_controller::z_dot_dot_ref_callback, this);	
	_psi_dot_dot_sub = _nh.subscribe("/firefly/planner/psi_dot_dot_ref", 0, &geometric_controller::psi_dot_dot_ref_callback, this);	

	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &geometric_controller::odom_callback, this);	
	_imu_sub = _nh.subscribe("/firefly/ground_truth/imu", 0, &geometric_controller::imu_callback, this);	

	_act_pub = _nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);
	_wrench_pub = _nh.advertise<geometry_msgs::Wrench>("/firefly/command/wrench", 1);


	_allocation_matrix.resize(4,motor_number);
	_allocation_matrix << C_T , C_T , C_T , C_T , C_T , C_T ,
						C_T*arm_length*sin(M_PI/6),  C_T*arm_length,  C_T*arm_length*sin(M_PI/6), -C_T*arm_length*sin(M_PI/6), -C_T*arm_length, -C_T*arm_length*sin(M_PI/6),
						C_T*arm_length*cos(M_PI/6),  0,  -C_T*arm_length*cos(M_PI/6),  -C_T*arm_length*cos(M_PI/6), 0, C_T*arm_length*cos(M_PI/6),
						C_Q, -C_Q, C_Q, -C_Q, C_Q, -C_Q;


	_first_imu = false;
	_first_odom = false;
	_control_on = false;

 	_R_enu2ned << 1,0,0,0,-1,0,0,0,-1; 
	_Ib << Ixx,0,0,0,Iyy,0,0,0,Izz;

/*	_Kp = 6;
	_Kv = 4.7;
	_KR << 3 , 0 , 0 , 0 , 3 , 0 , 0 , 0 , 0.035;
	_KW << 0.52 , 0 , 0 , 0 , 0.52 , 0 , 0 , 0 ,0.025;
	_Ki = 0.1;
	_Ki_R << 0.1 , 0 , 0 , 0 , 0.1 , 0 ,0 , 0 , 0.1;
*/

	_Kp = 5; //10
	_Kv = 5;
	_KR << 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 1;
	_KW << 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 1; //0.52
	_Ki = 0.2; //0.2
	_Ki_R << 0.1, 0 , 0 , 0 , 0.1 , 0 , 0 , 0 , 0.1; //0.1


	_Rb.setIdentity();
}


Eigen::Matrix3d skew(Eigen::Vector3d v){
	Eigen::Matrix3d skew;
	skew <<    0 , -v(2) , v(1),
			 v(2) , 0   , -v(0),
			-v(1) , v(0) ,    0;
	return skew;
}

Eigen::Vector3d v_operator( Eigen::Matrix3d skew_matrix ){
	Eigen::Vector3d v;
	v << skew_matrix(2,1) , skew_matrix(0,2) , skew_matrix(1,0);
	return v;
}

void geometric_controller::x_ref_callback( std_msgs::Float64 msg){
	_pos_ref[0] = msg.data;
}
void geometric_controller::y_ref_callback( std_msgs::Float64 msg){
	_pos_ref[1] = msg.data;
}
void geometric_controller::z_ref_callback( std_msgs::Float64 msg){
	_pos_ref[2] = msg.data;
}
void geometric_controller::psi_ref_callback( std_msgs::Float64 msg){
	_psi_ref = msg.data;
}



void geometric_controller::x_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[0] = msg.data;
}
void geometric_controller::y_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[1] = msg.data;
}
void geometric_controller::z_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[2] = msg.data;
}
void geometric_controller::psi_dot_ref_callback( std_msgs::Float64 msg){
	_psi_ref_dot = msg.data;
}


void geometric_controller::x_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[0] = msg.data;
}
void geometric_controller::y_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[1] = msg.data;
}
void geometric_controller::z_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[2] = msg.data;
}
void geometric_controller::psi_dot_dot_ref_callback( std_msgs::Float64 msg){
	_psi_ref_dot_dot = msg.data;
}




void geometric_controller::odom_callback( nav_msgs::Odometry odom ) {

    Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    Eigen::Vector3d vel_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z); 
	
    _p_b = _R_enu2ned*pos_enu;		//trasformazione da enu a ned -> pb è in ned
	_p_b_dot = _R_enu2ned*vel_enu;



	_first_odom = true;
}

void geometric_controller::imu_callback ( sensor_msgs::Imu imu ){
	Eigen::Vector3d omega_wenu_b(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z); //in realtà è omega_b_b in ENU
	Eigen::Vector3d omega_wned_b = _R_enu2ned.transpose()*omega_wenu_b;




//	_omega_b_b = _Rb.transpose()*omega_wned_b;				//NED
	_omega_b_b = omega_wned_b;				//NED

	
	Eigen::Quaterniond quat(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z);
	Eigen::Matrix3d R = quat.toRotationMatrix();

    Eigen::Matrix3d _Rb = _R_enu2ned.transpose()*R*_R_enu2ned;
	_first_imu = true;
}

void geometric_controller::ctrl_loop() {	
	ros::Rate rate(100);

	Eigen::Vector3d e_p , e_p_dot, e_p_int , e_R, e_W, e_int_R ;
	Eigen::Vector3d z_bd , x_bd , y_bd , omega_b_b_ref, omega_b_b_ref_dot , omega_b_b_ref_old , A;
	Eigen::Vector3d e3(0,0,1);
	Eigen::Matrix3d Rb_d , Rb_d_old , Rb_d_dot;
	Rb_d_old.setIdentity();
	omega_b_b_ref_dot.setZero();
	e_p_int.setZero();
	e_int_R.setZero();

	_u_T = 0;
	Eigen::VectorXd angular_velocities_sq(motor_number);
	Eigen::VectorXd control_input(4);
    mav_msgs::Actuators act_msg;
    act_msg.angular_velocities.resize(motor_number);


	std::cout << "Waiting for sensors" << endl;
	while (!_first_imu) rate.sleep();
	std::cout << "Take off" << endl;
	for (int i = 0 ; i < motor_number ; i++){
	    act_msg.angular_velocities[i] = 545.1;	
	}
	_act_pub.publish(act_msg);
	sleep(5.0);

	std::cout << "Control on" << endl;
	_control_on = true;

	while(ros::ok){

		A = -_Kp*e_p -_Ki*e_p_int - _Kv*e_p_dot - mass*gravity*e3 + mass*_pos_ref_dot_dot;
		z_bd = -A/A.norm();
		x_bd(0) = cos(_psi_ref);
		x_bd(1) = sin(_psi_ref);
		x_bd(2) = 0;
		y_bd = skew(z_bd)*x_bd/(skew(z_bd)*x_bd).norm();

		Rb_d.block<3,1>(0,0) = skew(y_bd)*z_bd; 
		Rb_d.block<3,1>(0,1) = y_bd;
		Rb_d.block<3,1>(0,2) = z_bd;


		Rb_d_dot = (Rb_d - Rb_d_old)*100;
		Rb_d_old = Rb_d;
		omega_b_b_ref_dot = (omega_b_b_ref - omega_b_b_ref_old)*100;
		omega_b_b_ref_old = omega_b_b_ref;

		cout<<"pos_ref:"<<endl;
		for(int i = 0 ; i < 3 ; i++){
			cout << _pos_ref[i] << endl; 
		}
		cout << endl;
						
		e_p = _p_b - _pos_ref;
		e_p_dot = _p_b_dot - _pos_ref_dot;

		e_R = 0.5*v_operator(Rb_d.transpose()*_Rb - _Rb.transpose()*Rb_d);
		
		omega_b_b_ref = v_operator(Rb_d.transpose()*Rb_d_dot);

		e_W = _omega_b_b - _Rb.transpose()*Rb_d*omega_b_b_ref;

		for(int i = 0 ; i < 3 ; i++){
			e_p_int[i] = e_p_int[i] + e_p[i]*Ts;
			e_int_R[i] = e_int_R[i] + e_R[i]*Ts;
		}

		_tau_b = -_KR*e_R -_Ki_R*e_int_R - _KW*e_W + skew(_omega_b_b)*_Ib*_omega_b_b - _Ib*( skew(_omega_b_b)*_Rb.transpose()*Rb_d*omega_b_b_ref - _Rb.transpose()*Rb_d*omega_b_b_ref_dot );

		_u_T = -A.transpose()*_Rb*e3;



		geometry_msgs::Wrench wrench_msg;
		wrench_msg.force.x = 0;
		wrench_msg.force.y = 0;
		wrench_msg.force.z = -_u_T;
		wrench_msg.torque.x = _tau_b[0];
		wrench_msg.torque.y = _tau_b[1];
		wrench_msg.torque.z = _tau_b[2];

		_wrench_pub.publish(wrench_msg);


		control_input[0] = _u_T;
		control_input[1] = _tau_b[0];
		control_input[2] = _tau_b[1];
		control_input[3] = _tau_b[2];
	

	//	for (int i=0 ; i<4 ; i++)
	//		std::cout <<control_input[i] << endl;
	//		std::cout <<e_p[i] << endl;
	//	std::cout <<endl;
	

		angular_velocities_sq =  _allocation_matrix.transpose()*(_allocation_matrix*_allocation_matrix.transpose()).inverse() * control_input;

		for (int i=0 ; i<motor_number ; i++){
	//		std::cout << angular_velocities_sq(i) << endl;
			if (angular_velocities_sq(i) >= 0) 
		    act_msg.angular_velocities[i] = sqrt(angular_velocities_sq(i));	
			if (angular_velocities_sq(i) < 0) 
		    act_msg.angular_velocities[i] = -sqrt(-angular_velocities_sq(i));	
		}
	
		_act_pub.publish(act_msg);



		rate.sleep();		
	}
}

void geometric_controller::run() {
	boost::thread ctrl_loop_t ( &geometric_controller::ctrl_loop, this);
	ros::spin();	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "geometric_controller");
	geometric_controller gc;
	gc.run();
	return 0;
}
