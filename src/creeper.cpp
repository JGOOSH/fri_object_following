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
 #include <std_msgs/Int32.h>

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

#define UPDATE_RATE 5
#define DISTANCE_BUFFER 1.5
#define PI 3.1415926

#define IDLE_ROTATE_DEG 15
#define IDLE_ROTATE_RADIANS ((IDLE_ROTATE_DEG / 180.0) * PI)

#define PERSON_FRAME_OF_REFERENCE "/map"
#define CAMERA_FRAME_OF_REFERENCE "/nav_kinect_rgb_optical_frame"
#define BASE_FRAME_OF_REFERENCE "/base_link"

int hand_com = 0;

// C++ is a sad, sad language

// Due to incomprehensible boost error messages with boost::bind, we make the person detector a global variable and call
// it's methods directly.
PersonDetector detector;

// Used for queueing messages which don't have the other message component yet.
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

/*
 * Callback specifically for accepting a pose from the detector, and pairing it with any backlogged clouds.
 */
void pose_callback(const PosePtr pose) {
	pose_backlog.push_back(pose);

	while(pose_backlog.size() >= 1 && cloud_backlog.size() >= 1) {
		detector_callback(pose_backlog.front(), cloud_backlog.front());
		pose_backlog.pop_front();
		cloud_backlog.pop_front();
	}
}

/*
 * Callback specifically for accepting a cloud from the detector, and pairing it with any backlogged poses.
 */
void cloud_callback(const CloudPtr cloud) {
	cloud_backlog.push_back(cloud);

	while(pose_backlog.size() >= 1 && cloud_backlog.size() >= 1) {
		detector_callback(pose_backlog.front(), cloud_backlog.front());
		pose_backlog.pop_front();
		cloud_backlog.pop_front();
	}
}

/*
 * Callback for hand detection command
 */
 void hand_callback(const std_msgs::Int32::ConstPtr& m) {
 	hand_com = m->data;
}


/*
 * Creates a dummy stamped pose which we can pass off to the transform listener. Sets the pose and reference frame,
 * and then sets the stamped time to now.
 */
geometry_msgs::PoseStamped create_dummy_stamped_pose(const geometry_msgs::Pose& pose, const std::string reference_frame_in) {
	geometry_msgs::PoseStamped stamped_pose;
	stamped_pose.pose = pose;
	stamped_pose.header.frame_id = reference_frame_in;
	stamped_pose.header.stamp = ros::Time::now();

	return stamped_pose;
}

/*
 * Computes the moev goal pose for the robot. The target pose is computed in the robot's camera frame of reference
 * (eg, CAMERA_FRAME_OF_REFERENCE), and implicitly represents a position which is on the line between the robot and
 * the person, and is distance_buffer away from the person.
 *
 * Eg, it moves to distance_buffer away from the person.
 */
geometry_msgs::PoseStamped create_move_goal_target(const geometry_msgs::Pose& target_pose, double distance_buffer) {
	// In the Camera frame of reference, Z is forward, X is right, and Y is up.

	// We compute the yaw as the arctan of x/z; the flipping of x/z is due to the flipped coordinate frame, where
	// z goes outward and x goes to the right.
	double target_yaw = atan2(target_pose.position.x, target_pose.position.z);

	// The distance is normal Euclidian Distance; to enforce the distance buffer, we subtract it from the
	// total distance and clamp to ensure it's >= 0.
	double tentative_distance = sqrt(pow(target_pose.position.x, 2) + pow(target_pose.position.z, 2));
	double target_distance = std::max(0.0, tentative_distance - DISTANCE_BUFFER);

	// Then, we compute the move goal with some straightforward trigonometry. Interestingly, the
	// orientation assumes 0 radians is on the X axis, so being "straight" implies an orientation with a
	// yaw of PI / 2.
	geometry_msgs::PoseStamped result;
	result.pose.position.x = target_distance * sin(target_yaw);
	result.pose.position.y = 0.0;
	result.pose.position.z = target_distance * cos(target_yaw);
	result.pose.orientation = tf::createQuaternionMsgFromYaw(PI / 2);

	// Initialize header information.
	result.header.frame_id = CAMERA_FRAME_OF_REFERENCE;
	result.header.stamp = ros::Time::now();

	return result;
}

/*
 * Creates a pose which corresponds to a counterclockwise rotation of rotate_amount radians.
 */
geometry_msgs::PoseStamped create_move_goal_rotate(float rotate_amount) {
	geometry_msgs::PoseStamped result;
	result.pose.position.x = 0.0;
	result.pose.position.y = 0.0;
	result.pose.position.z = 0.0;
	result.pose.orientation = tf::createQuaternionMsgFromYaw(rotate_amount);

	result.header.frame_id = BASE_FRAME_OF_REFERENCE;
	result.header.stamp = ros::Time::now();

	return result;
}

/*
 * Determines the next action given the global state of the program.
 *
 * Unfortunately, as we have a mishmash of global/nonglobal state, we have to pass in
 * the transform listener.
 */
boost::optional<geometry_msgs::PoseStamped> act(tf::TransformListener& transform_listener, ros::Time& last_update_time) {
	// Fetch person numero 0
	boost::optional<Person&> potential_person = detector.person_by_id(0);

	// If a person with index 0 doesn't exist yet, sit here uselessly.
	if(!potential_person.is_initialized()) {
		ROS_INFO("No person found, sleeping...");

		return boost::optional<geometry_msgs::PoseStamped>(create_move_goal_rotate(IDLE_ROTATE_RADIANS));
	}

	// If they do exist, extract them.
	Person& person = potential_person.get();

	// Don't do anything if this person's data is stale.
	if(person.last_update_time() <= last_update_time) {
		ROS_INFO("Person info is stale, sleeping...");

		return boost::optional<geometry_msgs::PoseStamped>(create_move_goal_rotate(IDLE_ROTATE_RADIANS));
	}

	// Update the last update time, otherwise.
	last_update_time = person.last_update_time();

	ROS_INFO("Person %ld fetched from the person detector...", person.uid());

	// Now, we need to convert the person's pose from the /map frame of reference
	// to the kinect frame of reference, so we use the transform listener.
	geometry_msgs::PoseStamped input_pose = create_dummy_stamped_pose(person.last_pose(), PERSON_FRAME_OF_REFERENCE);
	geometry_msgs::PoseStamped output_pose;

	ros::Rate tf_buffer_rate(20);
	bool tf_buffered = true;
	do { 
		tf_buffered = true;
		try {
			transform_listener.waitForTransform(CAMERA_FRAME_OF_REFERENCE, PERSON_FRAME_OF_REFERENCE, ros::Time(0), ros::Duration(5.0));
			transform_listener.transformPose(CAMERA_FRAME_OF_REFERENCE, input_pose, output_pose);
		} catch(const tf::TransformException& transformException) {
			ROS_WARN("TF transformation information not buffered yet, waiting...");

			tf_buffered = false;
		}

		tf_buffer_rate.sleep();
	} while(!tf_buffered);

	return create_move_goal_target(output_pose.pose, DISTANCE_BUFFER);
}

/*
 * Main entry point for the node.
 */
int main(int argc, char* argv[]) {
	// Initialize our connection to ROS
	ros::init(argc, argv, "creeper");
	ros::NodeHandle node_handle;

	ROS_INFO("Connection to ROS initialized...");

	// Add subscribers for both the pose and cloud topics.

	ros::Subscriber pose_subscriber = node_handle.subscribe(POSE_TOPIC, 10, pose_callback);
	ros::Subscriber cloud_subscriber = node_handle.subscribe(CLOUD_TOPIC, 10, cloud_callback);

	// Add subscriber for hand detection - signals when the node can start following the person
	ros::Subscriber hand_subscriber = node_handle.subscribe("follow_me", 10, hand_callback);

	// Create a debug publisher to explicitly output our target position at any given time.
	ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::PoseStamped>(TARGET_TOPIC, 10);

	ROS_INFO("Message synchronizer initialized...");

	// Initialize the update rate for the robot as well as the persistent transform listener, for transforming between
	// frames of reference.
	ros::Rate update_rate(UPDATE_RATE);
	tf::TransformListener transform_listener;

	// The action client responsible for causing motion.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);

	ROS_INFO("Transform and action listeners initialized...");

	// Spin once to initialize information
	ros::spinOnce();

	ros::Time last_update_time = ros::Time::now();

	// Now, we just repeatedly loop and then follow the first person in the detector.
	while(ros::ok()) {
		// Compute where exactly we want to go.
		boost::optional<geometry_msgs::PoseStamped> next_action = act(transform_listener, last_update_time);

		if(hand_com == 1 ){
			ROS_INFO("SIGNAL is 1 ");
		}else{
			ROS_INFO("SIGNAL is 0");
		}

		// Checks for hand gesture command before starting to follow a person
		// When value of hand_com is 1, then we have received information that it can start
		// following the person in front, otherwise do not change goal to another location
		if(next_action.is_initialized() && hand_com == 1) {
			// Compute the move base goal in the camera frame of reference.
			move_base_msgs::MoveBaseGoal move_goal;
			move_goal.target_pose = *next_action;

			//ROS_INFO(" -> Moving to relative offset (%f, -, %f)", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.z);

			// Publish where we tenatively want to go.
			target_publisher.publish(move_goal.target_pose);

			// Send the goal and wait 3 seconds, at most, for a response.
			action_client.sendGoal(move_goal);
			action_client.waitForResult(ros::Duration(3.0f));

			//ROS_INFO("Movement finished, moving onto next action.");
		} else {
			//ROS_INFO("No action found, waiting...");
		}
		ros::spinOnce();
		update_rate.sleep();
	}

	return 0;
}
