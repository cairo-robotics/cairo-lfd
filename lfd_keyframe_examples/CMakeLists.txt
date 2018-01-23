cmake_minimum_required(VERSION 2.8.3)
project(lfd_keyframe_examples)

find_package(catkin 
	REQUIRED
	COMPONENTS
	lfd_keyframe
)

catkin_package(
	CATKIN_DEPENDS 
	rospy
	lfd_keyframe
    lfd_data_processor
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
