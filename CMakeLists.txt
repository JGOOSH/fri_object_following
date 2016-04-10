cmake_minimum_required(VERSION 2.8.3)
project(fri_object_following)

# Include OpenCV
find_package(OpenCV)

# Include the Catkin Packages for moving the robot and general ROS packages.
find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	tf
	move_base_msgs
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	cv_bridge
	image_transport
)

# Declare this package as a catkin package
catkin_package()

# Build Info
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(creeper src/creeper.cpp)
target_link_libraries(creeper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
