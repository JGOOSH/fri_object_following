#ifndef SRC_PERSONCLASSIFIER_H_
#define SRC_PERSONCLASSIFIER_H_

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"

/*
 * Classifies random input clouds from the sensor as either a positive or negative match for
 * the person this classifier is associated with.
 */
class PersonClassifier {
public:
	/*
	 * Returns true if the person detailed in the point cloud is "close enough" to the current
	 * life form, and false otherwise.
	 */
	virtual bool is_equivalent(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud) = 0;

	// We have to override the destructor for some reason.
	virtual ~PersonClassifier();
};




#endif /* SRC_PERSONCLASSIFIER_H_ */
