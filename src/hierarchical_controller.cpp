#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"

//Include tf libraries							
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"


using namespace std;

class hierarchical_controller {
	public:
		hierarchical_controller();			
        void run();

        void odom_callback( nav_msgs::Odometry odom );
    
	private:
		ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Publisher _act_pub;

        double _x;
        double _y;
        double _z;
        double _phi;
        double _theta;
        double _psi;
        
};


hierarchical_controller::hierarchical_controller() {
	_odom_sub = _nh.subscribe("/firefly/ground_truth/odometry", 0, &hierarchical_controller::odom_callback, this);	
}

void hierarchical_controller::odom_callback( nav_msgs::Odometry odom ) {
    _x = odom.pose.pose.position.x;
    _y = odom.pose.pose.position.y;
    _z = odom.pose.pose.position.z;

	tf::Matrix3x3 R;		//extract rotation from transform
	R.setRotation(tf::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w));

    tf::Matrix3x3 R_enu2ned;
    R_enu2ned[0][0] = 1;
    R_enu2ned[1][0] = 0;
    R_enu2ned[2][0] = 0;
    R_enu2ned[0][1] = 0;
    R_enu2ned[1][1] = -1;
    R_enu2ned[2][1] = 0;
    R_enu2ned[0][2] = 0;
    R_enu2ned[1][2] = 0;
    R_enu2ned[2][2] = -1;

    tf::Vector3 pos_enu(_x,_y,_z);
    tf::Vector3 pos_ned = R_enu2ned*pos_enu;

    tf::Matrix3x3 R_ned2p = R_enu2ned.transpose()*R;

    _psi = atan2( R_ned2p[1][0] , R_ned2p[0][0] );
    _theta = atan2( -R_ned2p[2][0] , sqrt(R_ned2p[2][1]*R_ned2p[2][1] + R_ned2p[2][2]*R_ned2p[2][2]) );
    _phi = atan2( R_ned2p[2][1],R_ned2p[2][2] );

/*
    for(int i = 0 ; i < 3 ; i++){
        for(int j = 0 ; j < 3 ; j++){
            if (R[i][j] < 0.00000001)
                R[i][j] = 0;
        }
    }
*/

    cout << "x: " << pos_ned[0] << endl;  
    cout << "y: " << pos_ned[1] << endl;  
    cout << "z: " << pos_ned[2] << endl;  
    cout << "phi: " << _phi << endl;  
    cout << "theta: " << _theta << endl;  
    cout << "psi: " << _psi << endl;
    cout << R_ned2p[0][0] << " " << R_ned2p[0][1] << " " << R_ned2p[0][2] << endl;
    cout << R_ned2p[1][0] << " " << R_ned2p[1][1] << " " << R_ned2p[1][2] << endl;
    cout << R_ned2p[2][0] << " " << R_ned2p[2][1] << " " << R_ned2p[2][2] << endl << endl;
}


/*
void hierachical_controller::ctrl_loop() {	
	ros::Rate rate(50);
	while( !_first_fk ) usleep(0.1);

	float i_cmd[5];			//define initial position joint values
	i_cmd[0] = 0.0;
	i_cmd[1] = 0.65;
	i_cmd[2] = -1.9;
	i_cmd[3] = -0.3;
        i_cmd[4] = CLOSE_GRIPPER;
	goto_initial_position( i_cmd );

	int error_var = 0;
	double x_ref ,y_ref, z_ref ,yaw, pitch_ref;
	pitch_ref = 0;
	int gripper = 0;
	int steps = 10;

	string ln;

	while(ros::ok){
		cout << "Press enter to start the trajectory execution" << endl;
		getline(cin, ln);
		cout << "Reference -- x: " << _x_marker << "  y: " << _y_marker << "  z: "<< _z_marker <<" pitch(deg): " << pitch_ref*180/M_PI << endl; 
		
		solve_move(_x_marker*2/3,_y_marker*2/3,_z_marker,-60*M_PI/180,1,_q_in,steps);	//go to intermediate position to prepare for grasping
		sleep(7);
		
		solve_move(_x_marker,_y_marker,_z_marker+0.006,pitch_ref,1,_q_in,steps);		//go to the grasping position
		sleep(6);
		
		solve_move(_x_marker,_y_marker,_z_marker, pitch_ref,0,_q_in,steps);				//close gripper
		sleep(6); 

		solve_move(0,0.1,0.2,pitch_ref,0,_q_in,steps);									//rise
		sleep(6);

		solve_move(0.05,0.05,0.3,pitch_ref,0,_q_in,steps);								//celebration
		sleep(6); 
		solve_move(-0.05,0.05,0.3,pitch_ref,0,_q_in,steps);								
		sleep(6); 

		goto_initial_position( i_cmd );
	}
}
*/
void hierarchical_controller::run() {
//	boost::thread get_dirkin_t( &red_fury_2_invkin::get_dirkin, this);
//	boost::thread ctrl_loop_t ( &red_fury_2_invkin::ctrl_loop, this);
	ros::spin();	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "hierarchical_controller");
	hierarchical_controller hc;
	hc.run();
	return 0;
}
