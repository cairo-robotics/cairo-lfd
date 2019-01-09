cmake_minimum_required(VERSION 2.8.3)
project(lfd_experiments)

find_package(catkin 
	REQUIRED
	COMPONENTS
    rospy
	cairo_lfd
)

catkin_package(
	CATKIN_DEPENDS 
	rospy
	cairo_lfd
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
