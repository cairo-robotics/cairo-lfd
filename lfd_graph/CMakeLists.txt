cmake_minimum_required(VERSION 2.8.3)
project(lfd_graph)

find_package(catkin
	REQUIRED
	COMPONENTS
	rospy
	lfd_keyframe
    predicate_classification_msgs
    predicate_classification
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS
	rospy
	lfd_keyframe
  predicate_classification_msgs
  predicate_classification
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
