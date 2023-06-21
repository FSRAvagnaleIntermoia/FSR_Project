#include "ros/ros.h" 
#include "mav_msgs/Actuators.h"

const int prop_number = 6;

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv,"actuator_publisher");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1);

    mav_msgs::Actuators act_msg;
    act_msg.angular_velocities.resize(prop_number);

	float ang_vel;
    
	ros::Rate rate(10); 
	while ( ros::ok() ) {
		cout << "Insert motor speed" << endl;
		cin >> ang_vel;
	/*	for (int i = 0 ; i < prop_number ; i++){
		    act_msg.angular_velocities[i] = ang_vel;	
		}*/

	    act_msg.angular_velocities[0] = ang_vel;	
	    act_msg.angular_velocities[1] = ang_vel;	
	    act_msg.angular_velocities[2] = ang_vel;	
	    act_msg.angular_velocities[3] = ang_vel;	
	    act_msg.angular_velocities[4] = ang_vel;	
	    act_msg.angular_velocities[5] = ang_vel;	


		cmd_pub.publish(act_msg);
		rate.sleep();		
	}
	
	return 0;
}