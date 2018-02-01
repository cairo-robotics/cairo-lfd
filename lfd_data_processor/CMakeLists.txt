cmake_minimum_required(VERSION 2.8.3)
project(lfd_data_processor)

find_package(catkin 
	REQUIRED
	COMPONENTS
	rospy
    geometry_msgs
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS 
	rospy
    geometry_msgs
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
