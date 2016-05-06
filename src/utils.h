/*
 * utils.h (Non-class file)
 *
 * Represents some generic typedefs which can be used throughout the program.
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

/*
 * A typedef for a sensor Point Cloud pointer, as recieved from the human detector.
 */
typedef sensor_msgs::PointCloud2::ConstPtr CloudPtr;

/*
 * A typedef for a geometry stamped pose, as recieved from the human detector.
 */
typedef geometry_msgs::PoseStamped::ConstPtr PosePtr;

#endif /* SRC_UTILS_H_ */
