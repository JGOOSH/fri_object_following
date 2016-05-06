#ifndef SRC_PERSON_H_
#define SRC_PERSON_H_

#include "geometry_msgs/Pose.h"

#include "utils.h"

#define PERSON_STORED_POSE_COUNT 5

/*
 * Represents a "person", which is simply a unique entity being tracked by the Person Detector.
 */
class Person {
private:
	// The unique identifier for this person.
	unsigned long _uid;

	// The last time the position of this person was updated.
	ros::Time _last_update_time;

	// The last N poses where this person was detected.
	std::list<geometry_msgs::Pose> _last_poses;

public:
	/*
	 * Construct an empty instance of a person with nothing set. Apparently required for maps and
	 * other standard collections (which implicitly rely on it existing?)
	 */
	Person();

	/*
	 * Construct a new person with the provided unique identifier.
	 */
	Person(const unsigned long uid);

	virtual ~Person();

	/*
	 * Obtain the current UID of the person.
	 */
	unsigned long uid() { return _uid; }

	/*
	 * Obtain the last time this person was updated.
	 */
	ros::Time last_update_time() { return _last_update_time; }

	/*
	 * Obtain a set of the last poses this person had. Newer values are pushed to the front of the list.
	 */
	std::list<geometry_msgs::Pose>& last_poses() { return _last_poses; }

	/*
	 * Return the very latest pose pushed to this person.
	 */
	geometry_msgs::Pose& last_pose();

	/*
	 * Pushes a new pose to the set of last poses which this person has, and updates the last update time.
	 */
	void push_pose(const geometry_msgs::Pose& pose);
};

#endif /* SRC_PERSON_H_ */
