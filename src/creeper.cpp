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

#define POSE_TOPIC "/segbot_pcl_person_detector/human_poses"
#define CLOUD_TOPIC "/segbot_pcl_person_detector/human_clouds"
#define TARGET_TOPIC "/creeper/target_pose"

#define UPDATE_RATE 10
#define DISTANCE_BUFFER 1.0
#define PI 3.1415926

#define PERSON_FRAME_OF_REFERENCE "/map"
#define CAMERA_FRAME_OF_REFERENCE "/nav_kinect_rgb_optical_frame"

// C++ is a sad, sad language

PersonDetector detector;
std::list<PosePtr> pose_backlog;
std::list<CloudPtr> cloud_backlog;

/*
 * Called whenever a new human pose + cloud pair comes in.
 */
void detector_callback(const PosePtr pose, const CloudPtr cloud) {
	// Person is unused, but we grab it simply for potential future use.
	Person& updated_person = detector.handle_update(pose, cloud);

	ROS_INFO("Detector Pose: (%f, %f, %f) : (%f, %f, %f, %f)", (*pose).pose.position.x, (*pose).pose.position.y, (*pose).pose.position.z,
			(*pose).pose.orientation.w, (*pose).pose.orientation.x, (*pose).pose.orientation.z, (*pose).pose.orientation.z);
}

void pose_callback(const PosePtr pose) {
	pose_backlog.push_back(pose);

	while(pose_backlog.size() >= 1 && cloud_backlog.size() >= 1) {
		detector_callback(pose_backlog.front(), cloud_backlog.front());
		pose_backlog.pop_front();
		cloud_backlog.pop_front();
	}
}

void cloud_callback(const CloudPtr cloud) {
	cloud_backlog.push_back(cloud);

	while(pose_backlog.size() >= 1 && cloud_backlog.size() >= 1) {
		detector_callback(pose_backlog.front(), cloud_backlog.front());
		pose_backlog.pop_front();
		cloud_backlog.pop_front();
	}
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

	ROS_INFO("Connection to ROS initialized...");

	// Make dummy subscribers to try and get SOMETHING to work
	ros::Subscriber pose_subscriber = node_handle.subscribe(POSE_TOPIC, 10, pose_callback);
	ros::Subscriber cloud_subscriber = node_handle.subscribe(CLOUD_TOPIC, 10, cloud_callback);

	// Make a dummy publisher to show where exactly we intend to move.
	ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::PoseStamped>(TARGET_TOPIC, 10);

	ROS_INFO("Message synchronizer initialized...");

	// Now, we just repeatedly loop and then follow the first person in the detector.
	ros::Rate update_rate(UPDATE_RATE);
	tf::TransformListener transform_listener;

	// The action client responsible for causing motion.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);

	ROS_INFO("Transform and action listeners initialized...");

	// Spin once to initialize information
	ros::spinOnce();

	ros::Time last_update_time = ros::Time::now();

	while(ros::ok()) {
		// Fetch person numero 0
		boost::optional<Person&> potential_person = detector.person_by_id(0);

		// If a person with index 0 doesn't exist yet, sit here uselessly.
		if(!potential_person.is_initialized()) {
			ROS_INFO("No person found, sleeping...");

			ros::spinOnce();
			update_rate.sleep();
			continue;
		}

		// If they do exist, extract them.
		Person& person = potential_person.get();

		// Don't do anything if this person's data is stale.
		if(person.last_update_time() <= last_update_time) {
			ROS_INFO("Person info is stale, sleeping...");

			ros::spinOnce();
			update_rate.sleep();
			continue;
		}

		// Update the last update time, otherwise.
		last_update_time = person.last_update_time();

		ROS_INFO("Person %ld fetched from the person detector...", person.uid());

		// Now, we need to convert the person's pose from the /map frame of reference
		// to the kinect frame of reference, so we use the transform listener.
		geometry_msgs::PoseStamped input_pose = create_dummy_stamped_pose(person.last_pose(), PERSON_FRAME_OF_REFERENCE);
		geometry_msgs::PoseStamped output_pose;

		ROS_INFO("Input Pose: (%f, %f, %f) : (%f, %f, %f, %f)", input_pose.pose.position.x, input_pose.pose.position.y, input_pose.pose.position.z,
				input_pose.pose.orientation.w, input_pose.pose.orientation.x, input_pose.pose.orientation.z, input_pose.pose.orientation.z);

		transform_listener.transformPose(CAMERA_FRAME_OF_REFERENCE, input_pose, output_pose);
		transform_listener.waitForTransform(CAMERA_FRAME_OF_REFERENCE, PERSON_FRAME_OF_REFERENCE, ros::Time(0), ros::Duration(5.0));

		// Now, we finally have the person's pose relative to us!
		geometry_msgs::Pose relative_person_pose = output_pose.pose;

		ROS_INFO(" -> Person %ld found at pose (%f, %f, %f)", person.uid(),
				relative_person_pose.position.x, relative_person_pose.position.y, relative_person_pose.position.z);

		// Now, we generate the move base goal based on the relative person position.
		// We can take advantage of the fact that the relative pose is from the perspective
		// of the camera, so the distance is the magnitude of the (X, Y) of the pose, and the
		// angle is the arctan.
		move_base_msgs::MoveBaseGoal move_goal;
		move_goal.target_pose.header.stamp = ros::Time::now();
		move_goal.target_pose.header.frame_id = CAMERA_FRAME_OF_REFERENCE;

		// In this frame of reference, Z means forwards, X means left, and Y means up.
		// TODO: Not sure if we need to rotate and move forward on Z only, or if we can just
		// move X/Z

		// We compute the yaw as the arctan of y/x, as usual.
		double target_yaw = atan2(relative_person_pose.position.x, relative_person_pose.position.z);

		// We compute the distance as the magnitude of the X,Y coordinates.
		double tentative_distance = sqrt(pow(relative_person_pose.position.x, 2) + pow(relative_person_pose.position.z, 2));
		double target_distance = std::max(0.0, tentative_distance - DISTANCE_BUFFER);

		ROS_INFO(" -> Moving %f distance units with an angle of %f...", target_distance, target_yaw);

		// Subtract some from the distance to move to ensure that we don't move ONTO the lifeform
		move_goal.target_pose.pose.position.x = target_distance * sin(target_yaw);
		move_goal.target_pose.pose.position.y = 0.0;
		move_goal.target_pose.pose.position.z = target_distance * cos(target_yaw);
		move_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(PI / 2);

		ROS_INFO(" -> X: %f, Z: %f, Angle: %f", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.z, target_yaw);

		// Publish where we tenatively want to go.
		target_publisher.publish(move_goal.target_pose);

		// Send the goal and wait 3 seconds, at most, for a response.
		action_client.sendGoalAndWait(move_goal, ros::Duration(3.0f));

		ros::spinOnce();
	}

	return 0;
}
