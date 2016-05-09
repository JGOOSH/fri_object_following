#ifndef SRC_SHIRTCOLORPERSONCLASSIFIER_H_
#define SRC_SHIRTCOLORPERSONCLASSIFIER_H_

#include "utils.h"
#include "PersonClassifier.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"

#include <pcl/point_cloud.h>

#define PERSON_SHIRT_BOTTOM_RATIO 0.5
#define PERSON_SHIRT_TOP_RATIO 0.875

/*
 * A person classifier which takes the average color of the person's "shirt" in the point cloud,
 * determined by computing the relative height of the person and then taking the average
 */
class ShirtColorPersonClassifier : public PersonClassifier {
private:
	// The threshold used for determing if a specified color distance is a "match"
	float _color_distance_threshold;

	// The last color reading which is used for checking similarity.
	Eigen::Vector3i _last_color_reading;

public:
	/*
	 * Constructs a new shirt color classifier, using the given point cloud to obtain the initial shirt color.
	 *
	 * Determinces equivalence using the given "color" distance threshold; a cloud is considered a match if the
	 * 3-dimensional distance between the old color and measured color is within this distance threshold.
	 */
	ShirtColorPersonClassifier(const sensor_msgs::PointCloud2& initial_cloud, float distance_threshold);
	virtual ~ShirtColorPersonClassifier();

	bool is_equivalent(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud);
};

#endif /* SRC_SHIRTCOLORPERSONCLASSIFIER_H_ */
