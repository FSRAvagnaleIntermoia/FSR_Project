#include "ros/ros.h" 
#include "std_msgs/Float64.h"

const int prop_number = 6;

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv,"reference_publisher");
	ros::NodeHandle nh;
	ros::Publisher ref_pub = nh.advertise<std_msgs::Float64>("/firefly/reference", 1);

	std_msgs::Float64 psi_ref_msg;

    
	ros::Rate rate(10); 
	while ( ros::ok() ) {
		cout << "Insert reference psi" << endl;
		cin >> psi_ref_msg.data;
		ref_pub.publish(psi_ref_msg);
		rate.sleep();		
	}
	
	return 0;
}