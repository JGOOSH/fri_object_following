/*
 * PersonDetector.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: blacksmithgu
 */

#include "PersonDetector.h"

PersonDetector::PersonDetector() {

}

PersonDetector::~PersonDetector() {
	// TODO Auto-generated destructor stub
}

void PersonDetector::callback_new_pose(const geometry_msgs::PoseStamped::ConstPtr& pose) {

}

void PersonDetector::callback_new_cloud(const sensor_msgs::PointCloud2::ConstPtr& cloud) {

}

void handle_update(geometry_msgs::Pose::ConstPtr& pose, sensor_msgs::PointCloud2::ConstPtr& cloud) {

}
