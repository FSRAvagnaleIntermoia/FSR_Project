#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Wrench.h"
#include <fstream>
#include "ros/package.h"


//Include tf libraries							
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"


using namespace std;

const double gravity = 9.8;
const double Ixx = 0.0347563;
const double Iyy = 0.0458929;
const double Izz = 0.0977;
const double C_T = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
const double C_Q = 0.016*0.00000854858;  // M_i = k_m * F_i
const double arm_length = 0.215;
const int motor_number = 6;
const double Ts = 0.01;

const double uncertainty = 1;

class geometric_controller {
	public:
		geometric_controller();			
		void ctrl_loop();
        void run();

		void ref_callback(std_msgs::Float64MultiArray msg);

        void odom_callback( nav_msgs::Odometry odom );
		void imu_callback( sensor_msgs::Imu imu);


		void empty_txt();
		void log_data();

	private:
		ros::NodeHandle _nh;

        ros::Subscriber _odom_sub;
        ros::Subscriber _imu_sub;	
		ros::Subscriber _ref_sub;

        ros::Publisher _act_pub;

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

		double _mass;
	


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
		Eigen::Vector3d e_R;
		Eigen::Vector3d e_W;

		Eigen::Vector3d _tau_b;

		double _log_time;	//for data log
		double _psi;

		bool _first_imu;
		bool _first_odom;
		bool _first_ref;
		bool _control_on;
};



geometric_controller::geometric_controller() : _pos_ref(0,0,-1) , _psi_ref(0) , _pos_ref_dot(0,0,0) , _psi_ref_dot(0), _pos_ref_dot_dot(0,0,0) , _psi_ref_dot_dot(0) {

	_ref_sub = _nh.subscribe("/low_level_planner/reference_trajectory", 0, &geometric_controller::ref_callback, this);	

	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &geometric_controller::odom_callback, this);	
	_imu_sub = _nh.subscribe("/firefly/ground_truth/imu", 0, &geometric_controller::imu_callback, this);	

	_act_pub = _nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);


	_allocation_matrix.resize(4,motor_number);
	_allocation_matrix << C_T , C_T , C_T , C_T , C_T , C_T ,
						C_T*arm_length*sin(M_PI/6),  C_T*arm_length,  C_T*arm_length*sin(M_PI/6), -C_T*arm_length*sin(M_PI/6), -C_T*arm_length, -C_T*arm_length*sin(M_PI/6),
						C_T*arm_length*cos(M_PI/6),  0,  -C_T*arm_length*cos(M_PI/6),  -C_T*arm_length*cos(M_PI/6), 0, C_T*arm_length*cos(M_PI/6),
						C_Q, -C_Q, C_Q, -C_Q, C_Q, -C_Q;


	_first_imu = false;
	_first_odom = false;
	_control_on = false;

 	_R_enu2ned << 1,0,0,0,-1,0,0,0,-1; 

	_mass = 1.56779;
	_mass = _mass * uncertainty;
	_Ib << Ixx,0,0,0,Iyy,0,0,0,Izz;
	_Ib = _Ib * uncertainty;

	_Kp = 10;
	_Kv = 10;
	_KR << 3 , 0 , 0 , 0 , 3 , 0 , 0 , 0 , 0.03;
	_KW << 0.5 , 0 , 0 , 0 , 0.5 , 0 , 0 , 0 , 0.05;
	_Ki = 0.2;
	_Ki_R << 0.01 , 0 , 0 , 0 , 0.01 , 0 , 0 , 0 , 0.005;

	_Rb.setIdentity();
	cout << _Rb << endl;
	_log_time = 0;
	empty_txt();
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

void geometric_controller::ref_callback( std_msgs::Float64MultiArray msg){
	_pos_ref(0) = msg.data[0];
	_pos_ref(1) = msg.data[1];
	_pos_ref(2) = msg.data[2];
	_psi_ref = msg.data[3];	
	_pos_ref_dot(0) = msg.data[4];
	_pos_ref_dot(1) = msg.data[5];
	_pos_ref_dot(2) = msg.data[6];
	_psi_ref_dot = msg.data[7];	
	_pos_ref_dot_dot(0) = msg.data[8];
	_pos_ref_dot_dot(1) = msg.data[9];
	_pos_ref_dot_dot(2) = msg.data[10];
	_psi_ref_dot_dot = msg.data[11];		
	_first_ref = true;
}



void geometric_controller::odom_callback( nav_msgs::Odometry odom ) {

    Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);		//world ENU frame
    Eigen::Vector3d vel_b_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);   // The sensor gives the velocities in body ENU frame
	
    _p_b = _R_enu2ned*pos_enu;							//transform in world NED frame
	_p_b_dot = _Rb*_R_enu2ned*vel_b_enu;  				//transform in world NED frame (first in body NED then in world NED)	

	_first_odom = true;
}

void geometric_controller::imu_callback ( sensor_msgs::Imu imu ){
	Eigen::Vector3d omega_b_b_enu(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z);    //omega_b_b in body ENU frame
	 _omega_b_b = _R_enu2ned.transpose()*omega_b_b_enu;								     	//transform in NED body frane
	
	Eigen::Quaterniond quat(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z);
	Eigen::Matrix3d R = quat.toRotationMatrix();

    _Rb = _R_enu2ned.transpose()*R*_R_enu2ned;
	_first_imu = true;
}

void geometric_controller::ctrl_loop() {	
	ros::Rate rate(100);

	Eigen::Vector3d e_p , e_p_dot, e_p_int , e_int_R ;
	Eigen::Vector3d z_bd , x_bd , y_bd , omega_b_b_ref, omega_b_b_ref_dot , omega_b_b_ref_old , omega_b_b_ref_old_f , omega_b_b_ref_f , omega_b_b_ref_dot_f , omega_b_b_ref_dot_old_f , omega_b_b_ref_dot_old , A;
	Eigen::Vector3d e3(0,0,1);
	Eigen::Matrix3d Rb_d , Rb_d_old , Rb_d_dot , Rb_d_dot_f , Rb_d_dot_old_f, Rb_d_dot_old;
	Rb_d_old.setIdentity();
	omega_b_b_ref_dot.setZero();
	e_p_int.setZero();
	e_int_R.setZero();

	_u_T = 0;
	Eigen::VectorXd angular_velocities_sq(motor_number);
	Eigen::VectorXd control_input(4);
	control_input.setZero();

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


	Rb_d_old.setIdentity();
	Rb_d_dot_old.setZero();
	Rb_d_dot_old_f.setZero();

	omega_b_b_ref_old.setZero();
	omega_b_b_ref_old.setZero();
	omega_b_b_ref_old_f.setZero();	
	


	while(ros::ok){

		cout << "p_b:" << _p_b << endl << endl;
		cout << "p_b_dot:" << _p_b_dot << endl << endl;
		cout << "pos_ref:" << _pos_ref << endl << endl;
		cout << "e_p: " << endl << e_p << endl << endl;		
		cout << "Rb_d: " << endl << Rb_d << endl << endl;
		cout << "Rb: " << endl << _Rb << endl << endl;
		cout << "e_R: " << e_R << endl << endl;
		cout << "e_W: " << e_W << endl << endl;		
		cout << "omega_b_b:" << _omega_b_b << endl << endl;
		cout << "control input: " << endl << control_input << endl << endl;
						
		e_p = _p_b - _pos_ref;
		e_p_dot = _p_b_dot - _pos_ref_dot;
		e_p_int = e_p_int + e_p*Ts;
		A = -_Kp*e_p -_Ki*e_p_int - _Kv*e_p_dot - _mass*gravity*e3 + _mass*_pos_ref_dot_dot;

		z_bd = -A/A.norm();
		x_bd(0) = cos(_psi_ref);
		x_bd(1) = sin(_psi_ref);
		x_bd(2) = 0;
		y_bd = skew(z_bd)*x_bd/(skew(z_bd)*x_bd).norm();

		Rb_d.block<3,1>(0,0) = skew(y_bd)*z_bd; 
		Rb_d.block<3,1>(0,1) = y_bd;
		Rb_d.block<3,1>(0,2) = z_bd;


		e_R = 0.5*v_operator(Rb_d.transpose()*_Rb - _Rb.transpose()*Rb_d);
				
		Rb_d_dot = (Rb_d - Rb_d_old)/Ts;
		Rb_d_dot_f = 0.9048*Rb_d_dot_old_f + Rb_d_dot_old*0.009516;  	//frequenza di taglio a 10 hz
		Rb_d_dot_old_f = Rb_d_dot_f;
		Rb_d_dot_old = Rb_d_dot;
		Rb_d_old = Rb_d;

		omega_b_b_ref = v_operator(Rb_d.transpose()*Rb_d_dot_f);
	//	omega_b_b_ref = v_operator(Rb_d.transpose()*Rb_d_dot);

		omega_b_b_ref_dot = (omega_b_b_ref - omega_b_b_ref_old)/Ts;
		omega_b_b_ref_dot_f = 0.9048*omega_b_b_ref_dot_old_f + omega_b_b_ref_dot_old*0.009516;  	//frequenza di taglio a 10 hz
		omega_b_b_ref_dot_old_f = omega_b_b_ref_dot_f;
		omega_b_b_ref_dot_old = omega_b_b_ref_dot;
		omega_b_b_ref_old = omega_b_b_ref;


		e_W = _omega_b_b - _Rb.transpose()*Rb_d*omega_b_b_ref;
		e_int_R = e_int_R + e_R*Ts;






		_tau_b = -_KR*e_R -_Ki_R*e_int_R - _KW*e_W + skew(_omega_b_b)*_Ib*_omega_b_b -_Ib*( skew(_omega_b_b)*_Rb.transpose()*Rb_d*omega_b_b_ref - _Rb.transpose()*Rb_d*omega_b_b_ref_dot_f );
	//	_tau_b = -_KR*e_R -_Ki_R*e_int_R - _KW*e_W + skew(_omega_b_b)*_Ib*_omega_b_b -_Ib*( skew(_omega_b_b)*_Rb.transpose()*Rb_d*omega_b_b_ref - _Rb.transpose()*Rb_d*omega_b_b_ref_dot );


		_u_T = -A.transpose()*_Rb*e3;



		geometry_msgs::Wrench wrench_msg;
		wrench_msg.force.x = 0;
		wrench_msg.force.y = 0;
		wrench_msg.force.z = -_u_T;
		wrench_msg.torque.x = _tau_b[0];
		wrench_msg.torque.y = _tau_b[1];
		wrench_msg.torque.z = _tau_b[2];

		control_input[0] = _u_T;
		control_input[1] = _tau_b[0];
		control_input[2] = _tau_b[1];
		control_input[3] = _tau_b[2];
	
		angular_velocities_sq =  _allocation_matrix.transpose()*(_allocation_matrix*_allocation_matrix.transpose()).inverse() * control_input;

		for (int i=0 ; i<motor_number ; i++){
		//	std::cout << angular_velocities_sq(i) << endl;
			if (angular_velocities_sq(i) >= 0) {
		    	act_msg.angular_velocities[i] = sqrt(angular_velocities_sq(i));	
			}
			if (angular_velocities_sq(i) < 0) {
				cout << "negative motor velocity!" << endl;
		    	act_msg.angular_velocities[i] = 0;
			}
		}

			//FOR DATA LOGGING

		_psi = atan2( _Rb(1,0) , _Rb(0,0) );
		if (_first_ref == true &&  _log_time < 120){
			log_data();			
			_log_time = _log_time + Ts;	
		}	


		_act_pub.publish(act_msg);



		rate.sleep();		
	}
}



void geometric_controller::empty_txt(){

    // "svuota" i file di testo

	std::string pkg_loc = ros::package::getPath("fsr_pkg");
	
    ofstream file_pos_x(pkg_loc + "/data/pos_x.txt");
    file_pos_x << "";
    file_pos_x.close();
    ofstream file_pos_y(pkg_loc + "/data/pos_y.txt");
    file_pos_y << "";
    file_pos_y.close();
    ofstream file_pos_z(pkg_loc + "/data/pos_z.txt");
    file_pos_z << "";
    file_pos_z.close();

    ofstream file_pos_x_ref(pkg_loc + "/data/pos_x_ref.txt");
    file_pos_x_ref << "";
    file_pos_x_ref.close();
    ofstream file_pos_y_ref(pkg_loc + "/data/pos_y_ref.txt");
    file_pos_y_ref << "";
    file_pos_y_ref.close();
    ofstream file_pos_z_ref(pkg_loc + "/data/pos_z_ref.txt");
    file_pos_z_ref << "";
    file_pos_z_ref.close();		
    ofstream file_psi(pkg_loc + "/data/psi.txt");
    file_psi << "";
    file_psi.close();

    ofstream file_pos_x_err(pkg_loc + "/data/pos_x_err.txt");
    file_pos_x_err << "";
    file_pos_x_err.close();
    ofstream file_pos_y_err(pkg_loc + "/data/pos_y_err.txt");
    file_pos_y_err << "";
    file_pos_y_err.close();
    ofstream file_pos_z_err(pkg_loc + "/data/pos_z_err.txt");
    file_pos_z_err << "";
    file_pos_z_err.close();		
    ofstream file_psi_ref(pkg_loc + "/data/psi_ref.txt");
    file_psi_ref << "";
    file_psi_ref.close();				
	
    ofstream file_vel_x_err(pkg_loc + "/data/vel_x_err.txt");
    file_vel_x_err << "";
    file_vel_x_err.close();
    ofstream file_vel_y_err(pkg_loc + "/data/vel_y_err.txt");
    file_vel_y_err << "";
    file_vel_y_err.close();
    ofstream file_vel_z_err(pkg_loc + "/data/vel_z_err.txt");
    file_vel_z_err << "";
    file_vel_z_err.close();

    ofstream file_e_R_x(pkg_loc + "/data/e_R_x.txt");
    file_e_R_x << "";
    file_e_R_x.close();
    ofstream file_e_R_y(pkg_loc + "/data/e_R_y.txt");
    file_e_R_y << "";
    file_e_R_y.close();
    ofstream file_e_R_z(pkg_loc + "/data/e_R_z.txt");
    file_e_R_z << "";
    file_e_R_z.close();	

    ofstream file_e_W_x(pkg_loc + "/data/e_W_x.txt");
    file_e_W_x << "";
    file_e_W_x.close();
    ofstream file_e_W_y(pkg_loc + "/data/e_W_y.txt");
    file_e_W_y << "";
    file_e_W_y.close();
    ofstream file_e_W_z(pkg_loc + "/data/e_W_z.txt");
    file_e_W_z << "";
    file_e_W_z.close();		
	

    ofstream file_uT(pkg_loc + "/data/uT.txt");
    file_uT << "";
    file_uT.close();
    ofstream file_taub_x(pkg_loc + "/data/taub_x.txt");
    file_taub_x << "";
    file_taub_x.close();
    ofstream file_taub_y(pkg_loc + "/data/taub_y.txt");
    file_taub_y << "";
    file_taub_y.close();
    ofstream file_taub_z(pkg_loc + "/data/taub_z.txt");
    file_taub_z << "";
    file_taub_z.close();			

}


void geometric_controller::log_data(){
	cout << "logging" << endl;
	std::string pkg_loc = ros::package::getPath("fsr_pkg");

	ofstream file_x(pkg_loc + "/data/pos_x.txt",std::ios_base::app);
	file_x << _p_b(0) <<endl;
	file_x.close();
	ofstream file_y(pkg_loc + "/data/pos_y.txt",std::ios_base::app);
	file_y << _p_b(1) <<endl;
	file_y.close();
	ofstream file_z(pkg_loc + "/data/pos_z.txt",std::ios_base::app);
	file_z << _p_b(2) <<endl;
	file_z.close();							
	ofstream file_psi(pkg_loc + "/data/psi.txt",std::ios_base::app);
	file_psi << _psi <<endl;
	file_psi.close();				

	ofstream file_x_ref(pkg_loc + "/data/pos_x_ref.txt",std::ios_base::app);
	file_x_ref << _pos_ref(0) <<endl;
	file_x_ref.close();
	ofstream file_y_ref(pkg_loc + "/data/pos_y_ref.txt",std::ios_base::app);
	file_y_ref << _pos_ref(1) <<endl;
	file_y_ref.close();
	ofstream file_z_ref(pkg_loc + "/data/pos_z_ref.txt",std::ios_base::app);
	file_z_ref << _pos_ref(2) <<endl;
	file_z_ref.close();			
	ofstream file_psi_ref(pkg_loc + "/data/psi_ref.txt",std::ios_base::app);
	file_psi_ref << _psi_ref <<endl;
	file_psi_ref.close();	

	ofstream file_x_err(pkg_loc + "/data/pos_x_err.txt",std::ios_base::app);
	file_x_err << _p_b(0)-_pos_ref(0) <<endl;
	file_x_err.close();
	ofstream file_y_err(pkg_loc + "/data/pos_y_err.txt",std::ios_base::app);
	file_y_err << _p_b(1)-_pos_ref(1) <<endl;
	file_y_err.close();
	ofstream file_z_err(pkg_loc + "/data/pos_z_err.txt",std::ios_base::app);
	file_z_err << _p_b(2)-_pos_ref(2) <<endl;
	file_z_err.close();			

	ofstream file_vel_x_err(pkg_loc + "/data/vel_x_err.txt",std::ios_base::app);
	file_vel_x_err << _p_b_dot(0)-_pos_ref_dot(0) <<endl;
	file_vel_x_err.close();
	ofstream file_vel_y_err(pkg_loc + "/data/vel_y_err.txt",std::ios_base::app);
	file_vel_y_err << _p_b_dot(1)-_pos_ref_dot(1) <<endl;
	file_vel_y_err.close();
	ofstream file_vel_z_err(pkg_loc + "/data/vel_z_err.txt",std::ios_base::app);
	file_vel_z_err << _p_b_dot(2)-_pos_ref_dot(2) <<endl;
	file_vel_z_err.close();			

    ofstream file_uT(pkg_loc + "/data/uT.txt",std::ios_base::app);
    file_uT << _u_T << endl;
    file_uT.close();
    ofstream file_taub_x(pkg_loc + "/data/taub_x.txt",std::ios_base::app);
    file_taub_x << _tau_b(0) << endl;
    file_taub_x.close();
    ofstream file_taub_y(pkg_loc + "/data/taub_y.txt",std::ios_base::app);
    file_taub_y << _tau_b(1) << endl;
    file_taub_y.close();
    ofstream file_taub_z(pkg_loc + "/data/taub_z.txt",std::ios_base::app);
    file_taub_z << _tau_b(2) <<endl;
    file_taub_z.close();		

    ofstream file_e_R_x(pkg_loc + "/data/e_R_x.txt",std::ios_base::app);
    file_e_R_x << e_R(0) << endl;
    file_e_R_x.close();
    ofstream file_e_R_y(pkg_loc + "/data/e_R_y.txt",std::ios_base::app);
    file_e_R_y << e_R(1) << endl;
    file_e_R_y.close();
    ofstream file_e_R_z(pkg_loc + "/data/e_R_z.txt",std::ios_base::app);
    file_e_R_z << e_R(2) << endl;
    file_e_R_z.close();	

    ofstream file_e_W_x(pkg_loc + "/data/e_W_x.txt",std::ios_base::app);
    file_e_W_x << e_W(0) << endl;
    file_e_W_x.close();
    ofstream file_e_W_y(pkg_loc + "/data/e_W_y.txt",std::ios_base::app);
    file_e_W_y << e_W(1) << endl;
    file_e_W_y.close();
    ofstream file_e_W_z(pkg_loc + "/data/e_W_z.txt",std::ios_base::app);
    file_e_W_z << e_W(2) << endl;
    file_e_W_z.close();		

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
