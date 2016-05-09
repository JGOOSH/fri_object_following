#ifndef SRC_PERSONDETECTOR_H_
#define SRC_PERSONDETECTOR_H_

#include "Person.h"
#include "PersonClassifier.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

#include "utils.h"

/*
 * Class responsible for tracking a set of people by associating each with a PersonClassifier,
 * which can mark which person an input point cloud is most closely associated with.
 */
class PersonDetector {
private:

	/*
	 * The map of all currently tracked people.
	 */
	std::map<unsigned int, Person> _tracked;

	/*
	 * The map of all of the classifiers associated with each person.
	 */
	std::map<unsigned int, boost::shared_ptr<PersonClassifier> > _classifiers;

	// The next UID which we can assign to a new person.
	unsigned int _current_uid;

	// Attempts to classify the cloud as a person which is currently being tracked.
	boost::optional<Person&> classify_message(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud);

	// Given a pose and cloud, creates a new person.
	Person& create_person(const geometry_msgs::Pose& pose, const sensor_msgs::PointCloud2& cloud);
public:
	PersonDetector();
	virtual ~PersonDetector();

	// Actually handles the update tracking information using a pose and cloud.
	Person& handle_update(const PosePtr pose, const CloudPtr cloud);

	/*
	 * Returns the map of all currently tracked individuals.
	 */
	std::map<unsigned int, Person>& tracked() { return _tracked; }

	/*
	 * Returns the map of all active classifiers for tracked individuals.
	 */
	std::map<unsigned int, boost::shared_ptr<PersonClassifier> >& classifiers() { return _classifiers; }

	/*
	 * Returns a reference to the person with the specified ID, if they exist.
	 * Otherwise, returns an empty optional.
	 */
	boost::optional<Person&> person_by_id(unsigned int uid);

	/*
	 * Returns a reference to the classifier associated with the specified ID, if they exist.
	 * Otherwise, returns an empty optional.
	 */
	boost::optional<PersonClassifier&> classifier_by_id(unsigned int uid);
};

#endif /* SRC_PERSONDETECTOR_H_ */
