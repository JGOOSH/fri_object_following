# FRI Object Following

Final Project for CS378 (Autonomous Intelligence in Robotics), as of Spring 2016.

The _Object Following_ project allows for specific objects to be recognized and followed in the physical world
It is initialized by choosing the particular object/region to follow, and then tries to keep track of the
particular object it is following until told to stop, or the object is no longer visible for an extended
number of frames.

Current Roadmap:
- [x] Create Repository
- [x] Set up ROS Project Skeleton
- [ ] Determine how to do region selection
- [x] Find identifying information about region for tracking
- [x] Set up actual tracking and find "center of object" in any frame with some confidence
- [x] Set up actual robot motion and distance
- [x] Quality Assurance & Testing
- [x] Create Final Presentation Papers

Created by Victoria Zhou, Saket Sadani, and Michael Brenan.

How to Run:

1. Download utexas-bwi library (if not already installed). Instructions found at https://github.com/utexas-bwi/bwi
2. Download background_people_perception: git clone https://github.com/utexas-bwi/bwi_experimental into a workspace
2. Download this repo: git clone https://github.com/blacksmithgu/fri_object_following into workspace
3. catkin_make (in the workspace)
5. source devel/setup.bash
6. roslaunch roslaunch bwi_launch segbot_v2.launch
7. Localize the robot (give it it's 2D position estimate)
8. Run background_pcl_perception: roslaunch pcl_perception background_people_detection.launch
9. Run creeper: rosrun fri_object_following creeper
