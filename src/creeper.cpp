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

#include "utils.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define POSE_TOPIC "segbot_pcl_person_detector/human_poses"
#define CLOUD_TOPIC "segbot_pcl_person_detector/human_clouds"


PersonDetector detector;

// TODO: Add classifier back.
// TODO: Seriously. This doesn't work yet.
// TODO: RIP. RIP C++.
// TODO: Try using a real language, pal
void detector_callback(const PosePtr pose, const CloudPtr cloud) {
	detector.handle_update(pose, cloud);
}

/*
 * Main entry point for the node.
 */
int main(int argc, char* argv[]) {
	// Initialize our connection to ROS
	ros::init(argc, argv, "creeper");
	ros::NodeHandle node_handle;

	message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(node_handle, POSE_TOPIC, 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(node_handle, CLOUD_TOPIC, 10);

	// Then, set up our combining thingy.
	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> ApproxSyncPolicy;

	// Set up the synchronizer and register the callback.
	message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(10), sub_pose, sub_cloud);
	sync.registerCallback(boost::bind(detector_callback, _1, _2));

	// To be extra useful, do nothing and just spin.
	ros::spin();

	return 0;
}
