cmake_minimum_required(VERSION 2.8.3)
project(cairo_lfd)

## Compile as C++11, supported in ROS Kinetic and newer
##c++11 required for moveit
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  predicate_classification
  predicate_classification_msgs
  cairo_lfd_msgs
)

catkin_python_setup()

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   ConstraintWebTrigger.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
    CATKIN_DEPENDS 
    rospy
    std_msgs
    geometry_msgs
    predicate_classification
    predicate_classification_msgs
    cairo_lfd_msgs
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
