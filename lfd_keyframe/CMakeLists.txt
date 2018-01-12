cmake_minimum_required(VERSION 2.8.3)
project(lfd)

find_package(catkin 
	REQUIRED
	COMPONENTS
	rospy
	binary_predicate_classification
	binary_predicate_classification_msgs
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS 
	rospy
	binary_predicate_classification
	binary_predicate_classification_msgs
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
