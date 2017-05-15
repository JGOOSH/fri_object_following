#ifndef SRC_TRIVIALPERSONCLASSIFIER_H_
#define SRC_TRIVIALPERSONCLASSIFIER_H_

#include "PersonClassifier.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"

/*
 * A simple classifier which always returns true to a classification.
 */
class TrivialPersonClassifier : public PersonClassifier {
public:
	TrivialPersonClassifier();
	virtual ~TrivialPersonClassifier();

	bool is_equivalent(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud);
};

#endif /* SRC_TRIVIALPERSONCLASSIFIER_H_ */
