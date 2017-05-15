cmake_minimum_required(VERSION 2.8.3)
project(fri_object_following)

# Include PCL
find_package(PCL 1.7 REQUIRED)


# Include the Catkin Packages for moving the robot and general ROS packages.
find_package(catkin REQUIRED COMPONENTS
	roscpp
	actionlib
	actionlib_msgs
	tf
	move_base_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	image_transport
	pcl_conversions
	pcl_ros
)

# Declare this package as a catkin package
catkin_package()

# Build Info
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(creeper 
	src/creeper.cpp 
	src/Person.cpp 
	src/PersonDetector.cpp
	src/TrivialPersonClassifier.cpp
	src/ShirtColorPersonClassifier.cpp
	src/PersonClassifier.cpp)

target_link_libraries(creeper ${catkin_LIBRARIES} ${PCL_LIBRARIES})
