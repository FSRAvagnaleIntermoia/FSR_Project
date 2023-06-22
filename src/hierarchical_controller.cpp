#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "std_msgs/Float64.h"


//Include tf libraries							
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"


using namespace std;

const double gravity = 9.81;
const double mass = 1.56779;
const double Ixx = 0.0347563;
const double Iyy = 0.0458929;
const double Izz = 0.0977;
const double C_T = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
const double C_Q = 0.016*0.00000854858;  // M_i = k_m * F_i
const double arm_length = 0.215;
const int motor_number = 6;
const double Ts = 0.01;




class hierarchical_controller {
	public:
		hierarchical_controller();			
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

		tf::Vector3 _pos_ref;
		tf::Vector3 _eta_ref;
		tf::Vector3	_pos_ref_dot;
		tf::Vector3 _eta_ref_dot;
		tf::Vector3	_pos_ref_dot_dot;
		tf::Vector3 _eta_ref_dot_dot;

		tf::Matrix3x3 _Rb;
	    tf::Matrix3x3 _R_enu2ned;
		tf::Matrix3x3 _Q;
		tf::Matrix3x3 _Q_dot;
		tf::Matrix3x3 _Ib;
		tf::Matrix3x3 _C;	

		Eigen::Matrix4Xd _allocation_matrix;

		tf::Vector3 _omega_b_b;
		tf::Vector3 _p_b;
		tf::Vector3 _p_b_dot;
		tf::Vector3 _eta_b;
		tf::Vector3 _eta_b_dot;


//		tf::Matrix3x3 _Kp;
//		tf::Matrix3x3 _Ke;
		double _Kp;
		double _Kp_dot;
		double _Ke;
		double _Ke_dot;
		double _Ki;
		double _Ki_e;

		bool _first_imu;
		bool _first_odom;
};


tf::Matrix3x3 skew(tf::Vector3 v){
	tf::Matrix3x3 Skew;
	Skew[0][0] = 0;
	Skew[0][1] = -v[2];
	Skew[0][2] = v[1];
	Skew[1][0] = v[2];
	Skew[1][1] = 0;
	Skew[1][2] = -v[0];
	Skew[2][0] = -v[1];
	Skew[2][1] = v[0];
	Skew[2][2] = 0;
	return Skew;
}

hierarchical_controller::hierarchical_controller() : _pos_ref(0,0,-1) , _eta_ref(0,0,0) , _pos_ref_dot(0,0,0) , _eta_ref_dot(0,0,0), _pos_ref_dot_dot(0,0,0) , _eta_ref_dot_dot(0,0,0) , _R_enu2ned(1,0,0,0,-1,0,0,0,-1) , _Ib(Ixx,0,0,0,Iyy,0,0,0,Izz){

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
	_imu_sub = _nh.subscribe("/firefly/imu", 0, &hierarchical_controller::imu_callback, this);	
	_act_pub = _nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);

	_allocation_matrix.resize(4,motor_number);
	_allocation_matrix << C_T , C_T , C_T , C_T , C_T , C_T ,
						C_T*arm_length*sin(M_PI/6),  C_T*arm_length,  C_T*arm_length*sin(M_PI/6), -C_T*arm_length*sin(M_PI/6), -C_T*arm_length, -C_T*arm_length*sin(M_PI/6),
						C_T*arm_length*cos(M_PI/6),  0,  -C_T*arm_length*cos(M_PI/6),  -C_T*arm_length*cos(M_PI/6), 0, C_T*arm_length*cos(M_PI/6),
						C_Q,  -C_Q, C_Q,  -C_Q, C_Q, -C_Q;


	_first_imu = false;
	_first_odom = false;

	_Kp = 2;
	_Kp_dot = 1;
	_Ke = 10;
	_Ke_dot = 5;
	_Ki = 0.1;
	_Ki_e = 0.1;

}


void hierarchical_controller::x_ref_callback( std_msgs::Float64 msg){
	_pos_ref[0] = msg.data;
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
	double phi, theta, psi;

    tf::Vector3 pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    tf::Vector3 vel_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z); 
	
    _p_b = _R_enu2ned*pos_enu;		//trasformazione da enu a ned -> pb è in ned
	_p_b_dot = _R_enu2ned*vel_enu;
	
	tf::Matrix3x3 R(tf::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w));

    tf::Matrix3x3 R_ned2p = _R_enu2ned.transpose()*R*_R_enu2ned;

    psi = atan2( R_ned2p[1][0] , R_ned2p[0][0] );
    theta = atan2( -R_ned2p[2][0] , sqrt(R_ned2p[2][1]*R_ned2p[2][1] + R_ned2p[2][2]*R_ned2p[2][2]) );
    phi = atan2( R_ned2p[2][1],R_ned2p[2][2] );

	_eta_b[0] = phi;
	_eta_b[1] = theta;
	_eta_b[2] = psi;
	_Rb = R_ned2p;		//DA WORLD NED A BODY NED
	
	_first_odom = true;
}

void hierarchical_controller::imu_callback ( sensor_msgs::Imu imu ){
	tf::Vector3 omega_wenu_b(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z); 

	tf::Vector3 omega_wned_b = _R_enu2ned.transpose()*omega_wenu_b;

	_omega_b_b = _Rb.transpose()*omega_wned_b;				//NED

	double phi = _eta_b[0];
	double theta = _eta_b[1];
	double phi_dot = _eta_b_dot[0];
	double theta_dot = _eta_b_dot[1];

	_Q[0][0] = 1;
	_Q[0][1] = 0;
	_Q[0][2] = -sin(theta);
	_Q[1][0] = 0;
	_Q[1][1] = cos(phi);
	_Q[1][2] = cos(theta)*sin(phi);
	_Q[2][0] = 0;
	_Q[2][1] = -sin(phi);
	_Q[2][2] = cos(theta)*cos(phi);	

	_Q_dot[0][0] = 0;
	_Q_dot[0][1] = 0;
	_Q_dot[0][2] = -theta_dot*cos(theta);
	_Q_dot[1][0] = 0;
	_Q_dot[1][1] = -phi_dot*sin(phi);
	_Q_dot[1][2] = -theta_dot*sin(theta)*sin(phi) + phi_dot*cos(theta)*cos(phi);
	_Q_dot[2][0] = 0;
	_Q_dot[2][1] = -phi_dot*cos(phi);
	_Q_dot[2][2] = -theta_dot*sin(theta)*cos(phi) - phi_dot*cos(theta)*sin(phi);

	_eta_b_dot = _Q.inverse()*_omega_b_b;

	tf::Matrix3x3 C1, C2;
	C1 = _Q.transpose()*skew(_Q*_eta_b_dot)*_Ib*_Q;
	C2 = _Q.transpose()*_Ib*_Q_dot;
	for(int i = 0 ; i<3;i++){
		for(int j = 0 ; j<3;j++){
			_C[i][j] = C1[i][j] + C2[i][j];
		}
	}

	_first_imu = true;
}



void hierarchical_controller::ctrl_loop() {	
	ros::Rate rate(100);

	tf::Vector3 e_p , e_p_dot, e_p_int , e_eta, e_eta_dot, e_eta_int , mu_d , tau_tilde , tau_b ;
	for(int i = 0 ; i < 3 ; i++){
		e_p_int[i] = 0;
		e_eta_int[i] = 0;
	}


	double u_T = 0;
	Eigen::VectorXd angular_velocities_sq(motor_number);
	Eigen::VectorXd control_input(4);
    mav_msgs::Actuators act_msg;
    act_msg.angular_velocities.resize(motor_number);


	cout << "Waiting for sensors" << endl;
	while (!_first_imu && !_first_odom) rate.sleep();

	cout << "Take off" << endl;
	for (int i = 0 ; i < motor_number ; i++){
	    act_msg.angular_velocities[i] = 545.5;	
	}
	_act_pub.publish(act_msg);

	sleep(5.0);
	cout << "Control on" << endl;


	double phi_ref , theta_ref , phi_ref_dot, theta_ref_dot , phi_ref_dot_dot , theta_ref_dot_dot , phi_ref_old = 0 , theta_ref_old = 0 , phi_ref_dot_old = 0 , theta_ref_dot_old = 0 ;

	while(ros::ok){
		e_p = _p_b - _pos_ref;
		e_p_dot = _p_b_dot - _pos_ref_dot;
		for(int i = 0 ; i < 3 ; i++){
			e_p_int[i] = e_p_int[i] + e_p[i]*Ts;
			e_eta_int[i] = e_eta_int[i] + e_eta[i]*Ts;
		}

		mu_d = -_Kp*e_p -_Kp_dot*e_p_dot -_Ki*e_p_int  + _pos_ref_dot_dot;
		u_T = mass*sqrt(mu_d[0]*mu_d[0] + mu_d[1]*mu_d[1] + (mu_d[2]-gravity)*(mu_d[2]-gravity));

		for (int i=0 ; i<3 ; i++)
			cout <<_pos_ref[i] << endl;
		cout << _eta_ref[2]*180/M_PI << endl << endl;


		phi_ref = asin(mass/u_T*(mu_d[1]*cos(_eta_ref[2]) - mu_d[0]*sin(_eta_ref[2])));
		phi_ref_dot = (phi_ref-phi_ref_old)*100;  // Ts=0.01
		phi_ref_dot_dot = (phi_ref_dot-phi_ref_dot_old)*100;  // Ts=0.01

		theta_ref = atan((mu_d[0]*cos(_eta_ref[2]) + mu_d[1]*sin(_eta_ref[2]))/(mu_d[2]-gravity));
		theta_ref_dot = (theta_ref-theta_ref_old)*100;  // Ts=0.01
		theta_ref_dot_dot = (theta_ref_dot-theta_ref_dot_old)*100;  // Ts=0.01

		_eta_ref[0] = phi_ref;
		_eta_ref[1] = theta_ref;
		_eta_ref_dot[0] = phi_ref_dot;
		_eta_ref_dot[1] = theta_ref_dot;
//		_eta_ref_dot_dot[0] = phi_ref_dot_dot;
//		_eta_ref_dot_dot[1] = theta_ref_dot_dot;				

		e_eta = _eta_b - _eta_ref;
		e_eta_dot = _eta_b_dot - _eta_ref_dot;
		tau_tilde = -_Ke*e_eta - _Ke_dot*e_eta_dot - _Ki_e*e_eta_int + _eta_ref_dot_dot;
		tau_b = _Ib*_Q*tau_tilde + (_Q.transpose()).inverse()*_C*_eta_b_dot;

		phi_ref_old = phi_ref;
		theta_ref_old = theta_ref;
		phi_ref_dot_old = phi_ref_dot;
		theta_ref_dot_old = theta_ref_dot;		


		control_input[0] = u_T;
		control_input[1] = tau_b[0];
		control_input[2] = tau_b[1];
		control_input[3] = tau_b[2];

	//	for (int i=0 ; i<4 ; i++)
	//		cout <<control_input[i] << endl;
	//	cout <<endl;
	

		angular_velocities_sq =  _allocation_matrix.transpose()*(_allocation_matrix*_allocation_matrix.transpose()).inverse() * control_input;

		for (int i=0 ; i<motor_number ; i++){
	//		cout << angular_velocities_sq(i) << endl;
			if (angular_velocities_sq(i) >= 0) 
		    act_msg.angular_velocities[i] = sqrt(angular_velocities_sq(i));	
			if (angular_velocities_sq(i) < 0) 
		    act_msg.angular_velocities[i] = -sqrt(-angular_velocities_sq(i));	
		}
	
		_act_pub.publish(act_msg);

//		cout << "prova" << endl;
		rate.sleep();		
	}
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
