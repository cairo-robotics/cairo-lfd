cmake_minimum_required(VERSION 2.8.3)
project(lfd)

find_package(catkin 
	REQUIRED
	COMPONENTS
	lfd_keyframe
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS 
	rospy
	lfd_keyframe
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
