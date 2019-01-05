cmake_minimum_required(VERSION 2.8.3)
project(lfd_processor_examples)

find_package(catkin 
	REQUIRED
	COMPONENTS
    rospy
	lfd_processor
)

catkin_package(
	CATKIN_DEPENDS 
	rospy
	lfd_processor
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
