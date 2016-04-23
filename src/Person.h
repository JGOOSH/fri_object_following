/*
 *  Represents a detected and tracked person.
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */


#ifndef SRC_PERSON_H_
#define SRC_PERSON_H_

#include "geometry_msgs/Pose.h"

#include "utils.h"

#define PERSON_STORED_POSE_COUNT 5

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
	 * Construct a "default" person for the purposes of maps and all that pizazz?
	 */
	Person();

	/*
	 * Construct a new person with the provided unique identifier.
	 */
	Person(const unsigned long uid);

	virtual ~Person();

	// Obtain the current UID of the person.
	unsigned long uid() { return _uid; }

	// Obtain the last time this person was updated.
	ros::Time last_update_time() { return _last_update_time; }

	// Obtain a set of the last poses this person had. Newer valeus are pushed to the front of the list.
	std::list<geometry_msgs::Pose> last_poses() { return _last_poses; }

	// Pushes a new pose to the set of last poses which this person has, and updates the last update time.
	void push_pose(const geometry_msgs::Pose pose);
};

#endif /* SRC_PERSON_H_ */
