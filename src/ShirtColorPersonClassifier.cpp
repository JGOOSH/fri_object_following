#include "ShirtColorPersonClassifier.h"

#include <pcl_ros/impl/transforms.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>

#include <pcl_conversions/pcl_conversions.h>


Eigen::Vector3i average_color(const sensor_msgs::PointCloud2& cloud) {

	ROS_INFO("Starting to compute the average color of a sensor cloud...");

	// First, convert the point cloud 2 into a pcl point cloud.
	pcl::PCLPointCloud2 pcl_cloud;
	pcl_conversions::toPCL(cloud, pcl_cloud);

	pcl::PointCloud<pcl::PointXYZRGB> result_cloud;
	pcl::fromPCLPointCloud2(pcl_cloud, result_cloud);

	ROS_INFO("Successfully converted the pointcloud2 to a PCL point cloud w/ XYZ and RGB");

	Eigen::Vector3i current_sum(0, 0, 0);
	for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator iter = result_cloud.begin(); iter != result_cloud.end(); iter++)
		current_sum += iter->getRGBVector3i();

	return current_sum / result_cloud.size();
}

ShirtColorPersonClassifier::ShirtColorPersonClassifier(const sensor_msgs::PointCloud2& initial_cloud, float color_dist_threshold)
	: _color_distance_threshold(color_dist_threshold) {

	_last_color_reading = average_color(initial_cloud);
}

ShirtColorPersonClassifier::~ShirtColorPersonClassifier() { }

bool ShirtColorPersonClassifier::is_equivalent(const geometry_msgs::Pose&, const sensor_msgs::PointCloud2& cloud) {
	Eigen::Vector3i input_average_color = average_color(cloud);
	// Tnen, compute the "distance" between the last color vector and the current color vector.

	Eigen::Vector3i color_difference = _last_color_reading - input_average_color;

	ROS_INFO("Input Color: %d, %d, %d | Difference: %d, %d, %d",
		input_average_color(0), input_average_color(1), input_average_color(2),
		color_difference(0), color_difference(1), color_difference(2));

	// Then, if the length of the difference vector is less than the threshold, update the
	// last reading and return true.
	if(color_difference.norm() <= _color_distance_threshold) {
		_last_color_reading = input_average_color;
		return true;
	}

	return false;
}

