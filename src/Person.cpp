/*
 * Person.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */

#include "Person.h"

Person::Person(unsigned long uid) : uid(uid), last_update_time(ros::Time::now()) {

}

Person::~Person() {
	// Party or something
}

