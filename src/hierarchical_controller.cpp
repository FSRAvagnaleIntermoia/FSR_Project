#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "ros/package.h"
#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

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
const bool enable_estimator = 1;
const bool enable_disturbance = 1;

const double init_prop_speed = 546;
const double takeoff_time = 2;



class hierarchical_controller {
	public:
		hierarchical_controller();			
		void ctrl_loop();
//		void estimator_loop();
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


		void empty_txt();
		void log_data();

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
		Eigen::Vector3d _eta_ref;
		Eigen::Vector3d	_pos_ref_dot;
		Eigen::Vector3d _eta_ref_dot;
		Eigen::Vector3d	_pos_ref_dot_dot;
		Eigen::Vector3d _eta_ref_dot_dot;

		Eigen::Matrix3d _Rb;
	    Eigen::Matrix3d _R_enu2ned;
		Eigen::Matrix3d _Q;

		Eigen::Matrix3d _Q_dot;
		Eigen::Matrix3d _Ib;
		Eigen::Matrix3d _C;	
		Eigen::Matrix3d _M;



		Eigen::Matrix4Xd _allocation_matrix;

		Eigen::Vector3d _omega_b_b;
		Eigen::Vector3d _p_b;
		Eigen::Vector3d _p_b_dot;
		Eigen::Vector3d _eta_b;
		Eigen::Vector3d _eta_b_dot;


		Eigen::Vector3d _est_dist_lin;
		Eigen::Vector3d _est_dist_ang;

		double _mass;

//		tf::Matrix3x3 _Kp;
//		tf::Matrix3x3 _Ke;
		Eigen::Matrix3d _Ke;
		double _Kp;
		double _Kp_dot;
//		double _Ke;
//		double _Ke_dot;
		double _Ki;
//		double _Ki_e;

		Eigen::Matrix3d _Ke_dot;
		Eigen::Matrix3d _Ki_e;

		double _c0; //estimator constant


		double _u_T;
		Eigen::Vector3d _tau_b;
		double _log_time;	//for data log

		bool _first_imu;
		bool _first_odom;
		bool _first_ref;
		bool _control_on;
};



hierarchical_controller::hierarchical_controller() : _pos_ref(0,0,-1) , _eta_ref(0,0,0) , _pos_ref_dot(0,0,0) , _eta_ref_dot(0,0,0), _pos_ref_dot_dot(0,0,0) , _eta_ref_dot_dot(0,0,0){

	_x_sub = _nh.subscribe("/firefly/planner/x_ref", 0, &hierarchical_controller::x_ref_callback, this);	
	_y_sub = _nh.subscribe("/firefly/planner/y_ref", 0, &hierarchical_controller::y_ref_callback, this);	
	_z_sub = _nh.subscribe("/firefly/planner/z_ref", 0, &hierarchical_controller::z_ref_callback, this);	
	_psi_sub = _nh.subscribe("/firefly/planner/psi_ref", 0, &hierarchical_controller::psi_ref_callback, this);	
	_x_dot_sub = _nh.subscribe("/firefly/planner/x_dot_ref", 0, &hierarchical_controller::x_dot_ref_callback, this);	
	_y_dot_sub = _nh.subscribe("/firefly/planner/y_dot_ref", 0, &hierarchical_controller::y_dot_ref_callback, this);	
	_z_dot_sub = _nh.subscribe("/firefly/planner/z_dot_ref", 0, &hierarchical_controller::z_dot_ref_callback, this);	
	_psi_dot_sub = _nh.subscribe("/firefly/planner/psi_dot_ref", 0, &hierarchical_controller::psi_dot_ref_callback, this);	
	_x_dot_dot_sub = _nh.subscribe("/firefly/planner/x_dot_dot_ref", 0, &hierarchical_controller::x_dot_dot_ref_callback, this);	
	_y_dot_dot_sub = _nh.subscribe("/firefly/planner/y_dot_dot_ref", 0, &hierarchical_controller::y_dot_dot_ref_callback, this);	
	_z_dot_dot_sub = _nh.subscribe("/firefly/planner/z_dot_dot_ref", 0, &hierarchical_controller::z_dot_dot_ref_callback, this);	
	_psi_dot_dot_sub = _nh.subscribe("/firefly/planner/psi_dot_dot_ref", 0, &hierarchical_controller::psi_dot_dot_ref_callback, this);	

	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &hierarchical_controller::odom_callback, this);	
	_imu_sub = _nh.subscribe("/firefly/ground_truth/imu", 0, &hierarchical_controller::imu_callback, this);	

	_act_pub = _nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);
	_wrench_pub = _nh.advertise<geometry_msgs::Wrench>("/firefly/command/wrench", 1);


	_allocation_matrix.resize(4,motor_number);
	_allocation_matrix << C_T , C_T , C_T , C_T , C_T , C_T ,
						C_T*arm_length*sin(M_PI/6),  C_T*arm_length,  C_T*arm_length*sin(M_PI/6), -C_T*arm_length*sin(M_PI/6), -C_T*arm_length, -C_T*arm_length*sin(M_PI/6),
						C_T*arm_length*cos(M_PI/6),  0,  -C_T*arm_length*cos(M_PI/6),  -C_T*arm_length*cos(M_PI/6), 0, C_T*arm_length*cos(M_PI/6),
						C_Q,  -C_Q, C_Q,  -C_Q, C_Q, -C_Q;


	_first_imu = false;
	_first_odom = false;
	_control_on = false;

 	_R_enu2ned << 1,0,0,0,-1,0,0,0,-1; 

	_mass = 1.56779;
	_mass = _mass * uncertainty;
	_Ib << Ixx,0,0,0,Iyy,0,0,0,Izz;
	_Ib = _Ib * uncertainty;


	_Kp = 2;
	_Kp_dot = 1;
	_Ke << 50 , 0 , 0 , 0 , 50 , 0 , 0 , 0 , 25;
	_Ke_dot << 10 , 0 , 0 , 0 , 10 , 0 , 0 , 0 , 5;
	_Ki = 0.1;
	_Ki_e << 0.1 , 0 , 0 , 0 , 0.1 , 0 , 0 , 0 , 0.05;

	_c0 = 1;

	_Q.setIdentity();
	_Q_dot.setZero();
	_Rb.setIdentity();

	_est_dist_lin.setZero();
	_est_dist_ang.setZero();	


	empty_txt();
	_log_time = 0;
}

Eigen::Matrix3d skew(Eigen::Vector3d v){
	Eigen::Matrix3d skew;
	skew <<    0 , -v(2) , v(1),
			 v(2) , 0   , -v(0),
			-v(1) , v(0) ,    0;
	return skew;
}


void hierarchical_controller::x_ref_callback( std_msgs::Float64 msg){
	_pos_ref[0] = msg.data;
	_first_ref = true;
}
void hierarchical_controller::y_ref_callback( std_msgs::Float64 msg){
	_pos_ref[1] = msg.data;
}
void hierarchical_controller::z_ref_callback( std_msgs::Float64 msg){
	_pos_ref[2] = msg.data;
}
void hierarchical_controller::psi_ref_callback( std_msgs::Float64 msg){
	_eta_ref[2] = msg.data;
}



void hierarchical_controller::x_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[0] = msg.data;
}
void hierarchical_controller::y_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[1] = msg.data;
}
void hierarchical_controller::z_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot[2] = msg.data;
}
void hierarchical_controller::psi_dot_ref_callback( std_msgs::Float64 msg){
	_eta_ref_dot[2] = msg.data;
}


void hierarchical_controller::x_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[0] = msg.data;
}
void hierarchical_controller::y_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[1] = msg.data;
}
void hierarchical_controller::z_dot_dot_ref_callback( std_msgs::Float64 msg){
	_pos_ref_dot_dot[2] = msg.data;
}
void hierarchical_controller::psi_dot_dot_ref_callback( std_msgs::Float64 msg){
	_eta_ref_dot_dot[2] = msg.data;
}




void hierarchical_controller::odom_callback( nav_msgs::Odometry odom ) {

    Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);		//world ENU frame
    Eigen::Vector3d vel_b_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);   // The sensor gives the velocities in body ENU frame
	
    _p_b = _R_enu2ned*pos_enu;							//transform in world NED frame
	_p_b_dot = _Rb*_R_enu2ned*vel_b_enu;  				//transform in world NED frame (first in body NED then in world NED)	

	_first_odom = true;
}

void hierarchical_controller::imu_callback ( sensor_msgs::Imu imu ){
	Eigen::Vector3d omega_b_b_enu(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z);    //omega_b_b in body ENU frame
	 _omega_b_b = _R_enu2ned.transpose()*omega_b_b_enu;								     	//transform in NED body frane

	double phi, theta, psi;
	
	Eigen::Quaterniond quat(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z);
	Eigen::Matrix3d R = quat.toRotationMatrix();

    _Rb = _R_enu2ned.transpose()*R*_R_enu2ned;

    psi = atan2( _Rb(1,0) , _Rb(0,0) );
    theta = atan2( -_Rb(2,0) , sqrt(_Rb(2,1)*_Rb(2,1) + _Rb(2,2)*_Rb(2,2)) );
    phi = atan2( _Rb(2,1),_Rb(2,2) );

	_eta_b(0) = phi;
	_eta_b(1) = theta;
	_eta_b(2) = psi;

	double phi_dot = _eta_b_dot(0);
	double theta_dot = _eta_b_dot(1);


	_Q << 1 ,        0 	,  		   -sin(theta) ,
		  0 ,  cos(phi) ,  cos(theta)*sin(phi) ,
		  0 , -sin(phi) ,   cos(theta)*cos(phi);	


	_Q_dot << 0 ,        0           ,									         -theta_dot*cos(theta),
		      0 ,  -phi_dot*sin(phi) ,    -theta_dot*sin(theta)*sin(phi) + phi_dot*cos(theta)*cos(phi),
		      0 ,  -phi_dot*cos(phi) ,    -theta_dot*sin(theta)*cos(phi) - phi_dot*cos(theta)*sin(phi);	


	_eta_b_dot = _Q.inverse()*_omega_b_b;

	_C = _Q.transpose()*skew(_Q*_eta_b_dot)*_Ib*_Q 	+ _Q.transpose()*_Ib*_Q_dot ;
	_M = _Q.transpose()*_Ib*_Q;

	_first_imu = true;
}


bool applyWrench( const geometry_msgs::Wrench& wrench) {	//DISTURBANCE
    ros::NodeHandle nh;
    ros::ServiceClient applyWrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench srv;

    srv.request.body_name = "firefly/base_link";
    srv.request.reference_frame = "world";
    srv.request.reference_point.x = 0.0;
    srv.request.reference_point.y = 0.0;
    srv.request.reference_point.z = 0.0;
    srv.request.wrench = wrench;
    srv.request.wrench.force.y = -wrench.force.y;
    srv.request.wrench.force.z = -wrench.force.z;
    srv.request.wrench.torque.y = -wrench.torque.y;
    srv.request.wrench.torque.z = -wrench.torque.z;
    srv.request.start_time = ros::Time::now();
    srv.request.duration = ros::Duration(0.01);

    if (applyWrenchClient.call(srv) && srv.response.success)
    {
    //    ROS_INFO("Wrench applied successfully.");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to apply wrench.");
        return false;
    }
}
///////////



void hierarchical_controller::ctrl_loop() {	

//G=k0^2/(k0+s)^2; transfer function for the estimator
	ros::Rate rate(100);
    mav_msgs::Actuators act_msg;
    act_msg.angular_velocities.resize(motor_number);

	Eigen::VectorXd angular_velocities_sq(motor_number);
	angular_velocities_sq.setZero();
	Eigen::Vector4d control_input;
	control_input.setZero();

	double k0 = _c0;
	Eigen::Vector3d e3(0,0,1);
	Eigen::Vector3d q_lin , q_ang , q_dot_lin_est , q_dot_ang_est , q_lin_est , q_ang_est;	
	q_lin_est.setZero();
	q_ang_est.setZero();		
	_est_dist_lin.setZero();
	_est_dist_ang.setZero();	

	_u_T = 0;
	_tau_b.setZero();

	Eigen::Vector3d e_p , e_p_dot, e_p_int , e_eta, e_eta_dot, e_eta_int , mu_d , tau_tilde ;
	e_p_int.setZero();
	e_eta_int.setZero();


	double phi_ref , theta_ref , phi_ref_dot, theta_ref_dot , phi_ref_dot_dot , theta_ref_dot_dot , phi_ref_old = 0 , theta_ref_old = 0 , phi_ref_dot_old = 0 , theta_ref_dot_old = 0 ;
	float phi_ref_dot_dot_f = 0.0;
	float theta_ref_dot_dot_f = 0.0;
	float phi_ref_dot_dot_old_f = 0.0;
	float theta_ref_dot_dot_old_f = 0.0;
	float phi_ref_dot_dot_old = 0.0;
	float theta_ref_dot_dot_old = 0.0;	
	


	cout << "_est_dist_lin" << _est_dist_lin << endl;
	cout << "_est_dist_ang" << _est_dist_ang << endl;

	cout << "q_dot_lin_est" << q_dot_lin_est << endl;
	cout << "q_dot_ang_est" << q_dot_ang_est << endl;


	cout << "Waiting for sensors" << endl;
	while( !(_first_imu && _first_odom) ) rate.sleep();



	double time = 0;

	std::cout << "Take off" << endl;



	while(ros::ok){

		
		if (enable_estimator){						//ESTIMATION
			q_lin = _mass*_p_b_dot;
			q_ang = _M*_eta_b_dot;
			
			q_dot_lin_est = _est_dist_lin + _mass*gravity*e3 - _u_T*_Rb*e3;
			q_dot_ang_est = _est_dist_ang + _C.transpose()*_eta_b_dot + _Q.transpose()*_tau_b;
			
			q_lin_est = q_lin_est + q_dot_lin_est*Ts;
			q_ang_est = q_ang_est + q_dot_ang_est*Ts;

			_est_dist_lin = k0*(q_lin - q_lin_est);
			_est_dist_ang = k0*(q_ang - q_ang_est);

			cout << "_est_dist_lin: " << endl << _est_dist_lin << endl;
			cout << "_est_dist_ang: " << endl << _est_dist_ang << endl << endl;
		}

		if (time <= takeoff_time){						//TAKE OFF
			for (int i = 0 ; i < motor_number ; i++){
	    		act_msg.angular_velocities[i] = init_prop_speed;	
			}
			_u_T = pow(init_prop_speed,2)*C_T*motor_number;
			_act_pub.publish(act_msg);	
		}


		if (time > takeoff_time){						//CONTROL LOOP
		/*
			cout << "p_b:" << _p_b << endl << endl;
			cout << "p_b_dot:" << _p_b_dot << endl << endl;
			cout << "pos_ref:" << _pos_ref << endl << endl;
			cout << "e_p: " << endl << e_p << endl << endl;

			cout << "omega_b_b:" << _omega_b_b << endl << endl;
			cout << "eta_b:" << _eta_b << endl << endl;
			cout << "eta_b_dot:" << _eta_b_dot << endl << endl;
			cout << "control input: " << endl << control_input << endl << endl;
			*/

			e_p = _p_b - _pos_ref;
			e_p_dot = _p_b_dot - _pos_ref_dot;

			e_p_int = e_p_int + e_p*Ts;
			

			mu_d = -_Kp*e_p -_Kp_dot*e_p_dot -_Ki*e_p_int  + _pos_ref_dot_dot - _est_dist_lin/_mass;
			_u_T = _mass*sqrt(mu_d(0)*mu_d(0) + mu_d(1)*mu_d(1) + (mu_d(2)-gravity)*(mu_d(2)-gravity));



			phi_ref = asin( _mass/_u_T*( mu_d(1)*cos(_eta_ref(2)) - mu_d(0)*sin(_eta_ref(2)) ) );
			phi_ref_dot = (phi_ref-phi_ref_old)/Ts;  // Ts=0.01
			phi_ref_dot_dot = (phi_ref_dot-phi_ref_dot_old)/Ts;  // Ts=0.01


			phi_ref_dot_dot_f = 0.9048*phi_ref_dot_dot_old_f + phi_ref_dot_dot_old*0.009516;  	//frequenza di taglio a 10 hz
			phi_ref_dot_dot_old_f = phi_ref_dot_dot_f;
			phi_ref_dot_dot_old = phi_ref_dot_dot;


			theta_ref = atan((mu_d(0)*cos(_eta_ref(2)) + mu_d(1)*sin(_eta_ref(2)))/(mu_d(2)-gravity));
			theta_ref_dot = (theta_ref-theta_ref_old)/Ts;  // Ts=0.01
			theta_ref_dot_dot = (theta_ref_dot-theta_ref_dot_old)/Ts;  // Ts=0.01


			theta_ref_dot_dot_f = 0.9048*theta_ref_dot_dot_old_f + theta_ref_dot_dot_old*0.009516;  	//frequenza di taglio a 10 hz
			theta_ref_dot_dot_old_f = theta_ref_dot_dot_f;
			theta_ref_dot_dot_old = theta_ref_dot_dot;		



			_eta_ref(0) = phi_ref;
			_eta_ref(1) = theta_ref;
			_eta_ref_dot(0) = phi_ref_dot;
			_eta_ref_dot(1) = theta_ref_dot;
			_eta_ref_dot_dot(0) = phi_ref_dot_dot_f;
			_eta_ref_dot_dot(1) = theta_ref_dot_dot_f;				

			e_eta = _eta_b - _eta_ref;
			e_eta_dot = _eta_b_dot - _eta_ref_dot;
			e_eta_int = e_eta_int + e_eta*Ts;

			tau_tilde =  -_Ke*e_eta - _Ke_dot*e_eta_dot - _Ki_e*e_eta_int + _eta_ref_dot_dot;

			Eigen::Vector3d _tau_b_real;
			_tau_b = _Ib*_Q*tau_tilde + (_Q.transpose()).inverse()*_C*_eta_b_dot -(_Q.transpose()).inverse()*_est_dist_ang;


		/*	geometry_msgs::Wrench wrench_msg;
			wrench_msg.force.x = 0;
			wrench_msg.force.y = 0;
			wrench_msg.force.z = -_u_T;
			wrench_msg.torque.x = _tau_b(0);
			wrench_msg.torque.y = _tau_b(1);
			wrench_msg.torque.z = _tau_b(2);
			_wrench_pub.publish(wrench_msg);
		
		*/
			phi_ref_old = phi_ref;
			theta_ref_old = theta_ref;
			phi_ref_dot_old = phi_ref_dot;
			theta_ref_dot_old = theta_ref_dot;		

			control_input(0) = _u_T;
			control_input(1) = _tau_b(0);
			control_input(2) = _tau_b(1);
			control_input(3) = _tau_b(2);
		
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

			if (_first_ref == true &&  _log_time < 60){
				log_data();			
				_log_time = _log_time + Ts;	
			}	

			_act_pub.publish(act_msg);
		}

		if (enable_disturbance){
			geometry_msgs::Vector3 force , torque;
			force.x = 0.2;
			force.y = 0.3;
			force.z = 0.0;
			torque.x = 0.0;
			torque.y = 0.0;
			torque.z = 0.02;
			geometry_msgs::Wrench wrench;
			wrench.force.x = force.x*std::min(time/takeoff_time , 1.0);
			wrench.force.y = force.y*std::min(time/takeoff_time , 1.0);
			wrench.force.z = force.z*std::min(time/takeoff_time , 1.0);
			wrench.torque.x = torque.x*std::min(time/takeoff_time , 1.0);
			wrench.torque.y = torque.y*std::min(time/takeoff_time , 1.0);
			wrench.torque.z = torque.z*std::min(time/takeoff_time , 1.0);			
			applyWrench(wrench);
		}


		time = time + Ts;
		rate.sleep();
	}

}


void hierarchical_controller::empty_txt(){

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
    ofstream file_phi(pkg_loc + "/data/phi.txt");
    file_phi << "";
    file_phi.close();		
    ofstream file_theta(pkg_loc + "/data/theta.txt");
    file_theta << "";
    file_theta.close();		
    ofstream file_psi(pkg_loc + "/data/psi.txt");
    file_psi << "";
    file_psi.close();				

    ofstream file_pos_x_ref(pkg_loc + "/data/pos_x_ref.txt");
    file_pos_x_ref << "";
    file_pos_x_ref.close();
    ofstream file_pos_y_ref(pkg_loc + "/data/pos_y_ref.txt");
    file_pos_y_ref << "";
    file_pos_y_ref.close();
    ofstream file_pos_z_ref(pkg_loc + "/data/pos_z_ref.txt");
    file_pos_z_ref << "";
    file_pos_z_ref.close();		
    ofstream file_psi_ref(pkg_loc + "/data/psi_ref.txt");
    file_psi_ref << "";
    file_psi_ref.close();				

    ofstream file_pos_x_err(pkg_loc + "/data/pos_x_err.txt");
    file_pos_x_err << "";
    file_pos_x_err.close();
    ofstream file_pos_y_err(pkg_loc + "/data/pos_y_err.txt");
    file_pos_y_err << "";
    file_pos_y_err.close();
    ofstream file_pos_z_err(pkg_loc + "/data/pos_z_err.txt");
    file_pos_z_err << "";
    file_pos_z_err.close();		
    ofstream file_psi_err(pkg_loc + "/data/psi_err.txt");
    file_psi_err << "";
    file_psi_err.close();		

    ofstream file_vel_x_err(pkg_loc + "/data/vel_x_err.txt");
    file_vel_x_err << "";
    file_vel_x_err.close();
    ofstream file_vel_y_err(pkg_loc + "/data/vel_y_err.txt");
    file_vel_y_err << "";
    file_vel_y_err.close();
    ofstream file_vel_z_err(pkg_loc + "/data/vel_z_err.txt");
    file_vel_z_err << "";
    file_vel_z_err.close();
    ofstream file_vel_psi_err(pkg_loc + "/data/vel_psi_err.txt");
    file_vel_psi_err << "";
    file_vel_psi_err.close();	

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

    ofstream file_est_dist_lin_x(pkg_loc + "/data/est_dist_lin_x.txt");
    file_est_dist_lin_x << "";
    file_est_dist_lin_x.close();
    ofstream file_est_dist_lin_y(pkg_loc + "/data/est_dist_lin_y.txt");
    file_est_dist_lin_y << "";
    file_est_dist_lin_y.close();
    ofstream file_est_dist_lin_z(pkg_loc + "/data/est_dist_lin_z.txt");
    file_est_dist_lin_z << "";
    file_est_dist_lin_z.close();

    ofstream file_est_dist_ang_x(pkg_loc + "/data/est_dist_ang_x.txt");
    file_est_dist_ang_x << "";
    file_est_dist_ang_x.close();
    ofstream file_est_dist_ang_y(pkg_loc + "/data/est_dist_ang_y.txt");
    file_est_dist_ang_y << "";
    file_est_dist_ang_y.close();
    ofstream file_est_dist_ang_z(pkg_loc + "/data/est_dist_ang_z.txt");
    file_est_dist_ang_z << "";
    file_est_dist_ang_z.close();
}


void hierarchical_controller::log_data(){
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
	ofstream file_phi(pkg_loc + "/data/phi.txt",std::ios_base::app);
	file_phi << _eta_b(0) <<endl;
	file_phi.close();		
	ofstream file_theta(pkg_loc + "/data/theta.txt",std::ios_base::app);
	file_theta << _eta_b(1) <<endl;
	file_theta.close();				
	ofstream file_psi(pkg_loc + "/data/psi.txt",std::ios_base::app);
	file_psi << _eta_b(2) <<endl;
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
	file_psi_ref << _eta_ref(2) <<endl;
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
	ofstream file_psi_err(pkg_loc + "/data/psi_err.txt",std::ios_base::app);
	file_psi_err << _eta_b(2)-_eta_ref(2) <<endl;
	file_psi_err.close();		
	
	ofstream file_vel_x_err(pkg_loc + "/data/vel_x_err.txt",std::ios_base::app);
	file_vel_x_err << _p_b_dot(0)-_pos_ref_dot(0) <<endl;
	file_vel_x_err.close();
	ofstream file_vel_y_err(pkg_loc + "/data/vel_y_err.txt",std::ios_base::app);
	file_vel_y_err << _p_b_dot(1)-_pos_ref_dot(1) <<endl;
	file_vel_y_err.close();
	ofstream file_vel_z_err(pkg_loc + "/data/vel_z_err.txt",std::ios_base::app);
	file_vel_z_err << _p_b_dot(2)-_pos_ref_dot(2) <<endl;
	file_vel_z_err.close();			
	ofstream file_vel_psi_err(pkg_loc + "/data/vel_psi_err.txt",std::ios_base::app);
	file_vel_psi_err << _eta_b_dot(2)-_eta_ref_dot(2) <<endl;
	file_vel_psi_err.close();		

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

    ofstream file_est_dist_lin_x(pkg_loc + "/data/est_dist_lin_x.txt",std::ios_base::app);
    file_est_dist_lin_x << _est_dist_lin(0) << endl ;
    file_est_dist_lin_x.close();
    ofstream file_est_dist_lin_y(pkg_loc + "/data/est_dist_lin_y.txt",std::ios_base::app);
    file_est_dist_lin_y << _est_dist_lin(1) << endl ;
    file_est_dist_lin_y.close();
    ofstream file_est_dist_lin_z(pkg_loc + "/data/est_dist_lin_z.txt",std::ios_base::app);
    file_est_dist_lin_z << _est_dist_lin(2) << endl ;
    file_est_dist_lin_z.close();

    ofstream file_est_dist_ang_x(pkg_loc + "/data/est_dist_ang_x.txt",std::ios_base::app);
    file_est_dist_ang_x << _est_dist_ang(0) << endl ;
    file_est_dist_ang_x.close();
    ofstream file_est_dist_ang_y(pkg_loc + "/data/est_dist_ang_y.txt",std::ios_base::app);
    file_est_dist_ang_y << _est_dist_ang(1) << endl ;
    file_est_dist_ang_y.close();
    ofstream file_est_dist_ang_z(pkg_loc + "/data/est_dist_ang_z.txt",std::ios_base::app);
    file_est_dist_ang_z << _est_dist_ang(2) << endl ;
    file_est_dist_ang_z.close();

}


void hierarchical_controller::run() {
	boost::thread ctrl_loop_t ( &hierarchical_controller::ctrl_loop, this);
	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "hierarchical_controller");
	hierarchical_controller hc;
	hc.run();
	return 0;
}
