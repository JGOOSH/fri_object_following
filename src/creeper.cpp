/*
 * Creeper: Follows an individual. Creepily, I suppose.
 *
 * The program repeatedly loops and does the following:
 * 1. Obtains current readings of life forms from the person detector.
 * 2. Future work: Try to map life forms to identification numbers, so we can track them over time.
 * 3. Pick the selected life form to follow, and then find the center point.
 * 4. Compute the distance and angle to life form, and then send movement command if life form is sufficiently far away.
 */
#include <ros/ros.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "PersonDetector.h"
#include "Person.h"

#define POSE_TOPIC "segbot_pcl_person_detector/human_poses"

/*
 * Main entry point for the node.
 */
int main(int argc, char* argv[]) {
	// Initialize our connection to ROS
	ros::init(argc, argv, "creeper");
	ros::NodeHandle node_handle;

	// Create the person detector
	PersonDetector detector;

	ros::Subscriber pose_sub =
			node_handle.subscribe(POSE_TOPIC, 10, &PersonDetector::callback_new_pose, &detector);

	// To be extra useful, do nothing and just spin.
	ros::spin();
}
