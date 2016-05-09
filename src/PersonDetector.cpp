/*
 * PersonDetector.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */

#include "PersonDetector.h"
#include "TrivialPersonClassifier.h"
#include "ShirtColorPersonClassifier.h"

#include <ros/ros.h>

PersonDetector::PersonDetector()
	: _current_uid(0) {

}

PersonDetector::~PersonDetector() {
	// TODO Auto-generated destructor stub
}

/*
 * Handles the update of an individual.
 */
Person& PersonDetector::handle_update(const PosePtr pose, const CloudPtr cloud) {
	// First, we need to find the person actually associated with the cloud, if any.
	boost::optional<Person&> victim = classify_message(pose->pose, *cloud);

	// If we don't find a person, we need to create a new person.
	if(!victim.is_initialized())
		return create_person(pose->pose, *cloud);
	else {
		victim->push_pose(pose->pose);
		return victim.get();
	}

}

/*
 * Attempts to find the person which is most closely related to the provided cloud.
 */
boost::optional<Person&> PersonDetector::classify_message(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud) {
	// First, we need to see if any of the classifiers state that we've seen this person before.
	for(std::map<unsigned int, Person>::iterator iter = tracked().begin(); iter != tracked().end(); iter++) {
		unsigned int uid = (*iter).first;
		boost::optional<PersonClassifier&> classifier = classifier_by_id(uid);

		// If the classifier doesn't exist, there isn't much we can do to classify.
		if(!classifier.is_initialized())
			continue;

		if(classifier.get().is_equivalent(pose, cloud))
			return boost::optional<Person&>((*iter).second);
	}

	return boost::none;
}

/*
 * Create a new person from the provided stamped pose and point cloud, and insert them into the map.
 */
Person& PersonDetector::create_person(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud) {
	// Make a new person with the trivial person classifier, for now, and then give them an initial pose.
	Person lifeform(_current_uid++);
	lifeform.push_pose(pose);

	boost::shared_ptr<PersonClassifier> classifier(new ShirtColorPersonClassifier(cloud, 20.0));

	// Then insert them.
	tracked()[lifeform.uid()] = lifeform;
	classifiers()[lifeform.uid()] = classifier;

	// Return a reference to the person in the map.
	return tracked()[lifeform.uid()];
}

/*
 * Attempt to find a person by UID. Returns a reference to the person if found, or an empty optional if not.
 */
boost::optional<Person&> PersonDetector::person_by_id(unsigned int uid) {
	if(tracked().count(uid) >= 1)
		return tracked()[uid];

	return boost::none;
}

/*
 * Attempt to find a classifier by UID. Returns a reference to the classifier if found, or an empty optional otherwise.
 */
boost::optional<PersonClassifier&> PersonDetector::classifier_by_id(unsigned int uid) {
	if(classifiers().count(uid) >= 1)
		return *classifiers()[uid];

	return boost::none;
}
