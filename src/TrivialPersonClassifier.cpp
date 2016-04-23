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
	// TODO Auto-generated destructor stub
}

bool TrivialPersonClassifier::is_equivalent(const sensor_msgs::PointCloud2& cloud) {
	return true;
}

