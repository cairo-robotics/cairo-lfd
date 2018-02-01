cmake_minimum_required(VERSION 2.8.3)
project(lfd_analyzer)

find_package(catkin 
	REQUIRED
	COMPONENTS
	rospy
    lfd_data_processor
    predicate_classification
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS 
	rospy
    lfd_data_processor
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
