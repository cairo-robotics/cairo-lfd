cmake_minimum_required(VERSION 2.8.3)
project(lfd_processor)


find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  predicate_classification
  predicate_classification_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS 
    rospy
    geometry_msgs
    predicate_classification
    predicate_classification_msgs
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test_moveit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
