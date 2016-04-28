/*
 * TrivialPersonClassifier.h
 *
 *  Created on: Apr 23, 2016
 *      Author: blacksmithgu
 */

#ifndef SRC_TRIVIALPERSONCLASSIFIER_H_
#define SRC_TRIVIALPERSONCLASSIFIER_H_

#include "PersonClassifier.h"
#include "sensor_msgs/PointCloud2.h"

class TrivialPersonClassifier : public PersonClassifier {
public:
	TrivialPersonClassifier();
	virtual ~TrivialPersonClassifier();

	bool is_equivalent(const sensor_msgs::PointCloud2& cloud);
};

#endif /* SRC_TRIVIALPERSONCLASSIFIER_H_ */
