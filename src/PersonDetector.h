/*
 * PersonDetector.h
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */

#ifndef SRC_PERSONDETECTOR_H_
#define SRC_PERSONDETECTOR_H_

#include "Person.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

class PersonDetector {
public:
	PersonDetector();
	virtual ~PersonDetector();

	void callback_new_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void callback_new_cloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

	// Actually handles the update tracking information using a pose and cloud.
	void handle_update(geometry_msgs::Pose::ConstPtr& pose, sensor_msgs::PointCloud2::ConstPtr& cloud);
};

#endif /* SRC_PERSONDETECTOR_H_ */
