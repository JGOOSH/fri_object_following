/*
 *  Represents a detected and tracked person.
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */


#ifndef SRC_PERSON_H_
#define SRC_PERSON_H_

#include "geometry_msgs/Pose.h"

class Person {
private:
	// The unique identifier for this person.
	unsigned long uid;

	// The last time the position of this person was updated.
	ros::Time last_update_time;

	// The last N poses where this person was detected.
	std::vector<geometry_msgs::Pose> last_poses;

public:
	/*
	 * Construct a new person with the provided unique identifier.
	 */
	Person(unsigned long uid);

	virtual ~Person();
};

#endif /* SRC_PERSON_H_ */
