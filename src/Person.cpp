/*
 * Person.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */

#include "Person.h"

Person::Person() : _uid(0), _last_update_time(ros::Time::now()) {}

Person::Person(const unsigned long uid)
	: _uid(uid), _last_update_time(ros::Time::now()) { }

Person::~Person() {
	// Party or something
}

void Person::push_pose(const geometry_msgs::Pose pose) {
	// If our list is full, pop from the back.
	if(last_poses().size() >= PERSON_STORED_POSE_COUNT)
		last_poses().pop_back();

	// Push the new pose.
	last_poses().push_front(pose);

	// Then, update the update time.
	_last_update_time = ros::Time::now();
}
