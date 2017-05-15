/*
 * TrivialPersonClassifier.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: blacksmithgu
 */

#include "TrivialPersonClassifier.h"

/*
 * Default constructor which does nothing.
 */
TrivialPersonClassifier::TrivialPersonClassifier() {

}

/*
 * Default destructor which does nothing.
 */
TrivialPersonClassifier::~TrivialPersonClassifier() {

}

/*
 * The most advanced equivalence checker of all time. Always returns true.
 */
bool TrivialPersonClassifier::is_equivalent(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud) {
	return true;
}

