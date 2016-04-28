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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include "PersonDetector.h"
#include "Person.h"
#include "utils.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define POSE_TOPIC "segbot_pcl_person_detector/human_poses"
#define CLOUD_TOPIC "segbot_pcl_person_detector/human_clouds"

#define UPDATE_RATE 10
#define DISTANCE_BUFFER 1.0

#define PERSON_FRAME_OF_REFERENCE "/map"
#define CAMERA_FRAME_OF_REFERENCE "/nav_kinect_rgb_optical_frame"


PersonDetector detector;

// TODO: Add classifier back.
// TODO: Seriously. This doesn't work yet.
// TODO: RIP. RIP C++.
// TODO: Try using a real language, pal
void detector_callback(const PosePtr pose, const CloudPtr cloud) {
	// Person is unused, but we grab it simply for potential future use.
	Person& updated_person = detector.handle_update(pose, cloud);
}

/*
 * Creates a dummy stamped pose which we can pass off to the transform listener.
 */
geometry_msgs::PoseStamped create_dummy_stamped_pose(const geometry_msgs::Pose& pose, const std::string reference_frame_in) {
	geometry_msgs::PoseStamped stamped_pose;
	stamped_pose.pose = pose;
	stamped_pose.header.frame_id = reference_frame_in;
	stamped_pose.header.stamp = ros::Time(0);

	return stamped_pose;
}

/*
 * Main entry point for the node.
 */
int main(int argc, char* argv[]) {
	// Initialize our connection to ROS
	ros::init(argc, argv, "creeper");
	ros::NodeHandle node_handle;

	// Create subscribers to point cloud / human pose information.
	message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(node_handle, POSE_TOPIC, 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(node_handle, CLOUD_TOPIC, 10);

	// Then, set up our combining thingy.
	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> ApproxSyncPolicy;

	// Set up the synchronizer and register the callback.
	message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(10), sub_pose, sub_cloud);
	sync.registerCallback(boost::bind(detector_callback, _1, _2));

	// Now, we just repeatedly loop and then follow the first person in the detector.
	ros::Rate update_rate(UPDATE_RATE);
	tf::TransformListener transform_listener;

	// The action client responsible for causing motion.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);

	// Spin once to initialize information
	ros::spinOnce();

	while(ros::ok()) {
		// Fetch person numero 0
		boost::optional<Person&> potential_person = detector.person_by_id(0);

		// If a person with index 0 doesn't exist yet, sit here uselessly.
		if(!potential_person.is_initialized()) {
			ROS_INFO("No person found, sleeping...");
			update_rate.sleep();
			continue;
		}

		// If they do exist, extract them.
		Person& person = potential_person.get();


		// Now, we need to convert the person's pose from the /map frame of reference
		// to the kinect frame of reference, so we use the transform listener.
		geometry_msgs::PoseStamped input_pose = create_dummy_stamped_pose(person.last_pose(), PERSON_FRAME_OF_REFERENCE);
		geometry_msgs::PoseStamped output_pose;
		transform_listener.waitForTransform(CAMERA_FRAME_OF_REFERENCE, PERSON_FRAME_OF_REFERENCE, ros::Time(0), ros::Duration(5.0));
		transform_listener.transformPose(CAMERA_FRAME_OF_REFERENCE, input_pose, output_pose);

		// Now, we finally have the person's pose relative to us!
		geometry_msgs::Pose relative_person_pose = output_pose.pose;

		ROS_INFO("Person %ld found at pose (%f, %f, %f)", person.uid(),
				relative_person_pose.position.x, relative_person_pose.position.y, relative_person_pose.position.z);

		// Now, we generate the move base goal based on the relative person position.
		// We can take advantage of the fact that the relative pose is from the perspective
		// of the camera, so the distance is the magnitude of the (X, Y) of the pose, and the
		// angle is the arctan.
		move_base_msgs::MoveBaseGoal move_goal;
		move_goal.target_pose.header.stamp = ros::Time::now();
		move_goal.target_pose.header.frame_id = CAMERA_FRAME_OF_REFERENCE;

		// In this frame of reference, X means forwards, Y means left, and Z means up.
		// TODO: Not sure if we need to rotate and move forward on X only, or if we can just
		// move X/Y
		// For now, we assume that only X/Yaw works.

		// We compute the yaw as the arctan of y/x, as usual.
		double target_yaw = atan2(relative_person_pose.position.y, relative_person_pose.position.x);

		// We compute the distance as the magnitude of the X,Y coordinates.
		double tentative_distance = sqrt(pow(relative_person_pose.position.x, 2) + pow(relative_person_pose.position.y, 2));
		double target_distance = std::max(0.0, tentative_distance - DISTANCE_BUFFER);

		ROS_INFO("Moving %f distance units with an angle of %f...", target_distance, target_yaw);

		// Subtract some from the distance to move to ensure that we don't move ONTO the lifeform
		move_goal.target_pose.pose.position.x = target_distance;
		move_goal.target_pose.pose.position.y = 0.0;
		move_goal.target_pose.pose.position.z = 0.0;
		move_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

		// Send the goal and wait 5 seconds, at most, for a response.
		action_client.sendGoal(move_goal);
		action_client.waitForResult(ros::Duration(5.0f));
	}

	return 0;
}
