#include <ros/ros.h>

/*
 * Main entry point for the node.
 */
int main(int argc, char* argv[]) {
	// Initialize our connection to ROS
	ros::init(argc, argv, "creeper");
	ros::NodeHandle node_handle;

	// To be extra useful, do nothing and just spin.
	ros::spin();
}
