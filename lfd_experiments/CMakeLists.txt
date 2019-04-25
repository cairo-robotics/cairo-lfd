cmake_minimum_required(VERSION 2.8.3)
project(lfd_experiments)

find_package(catkin REQUIRED COMPONENTS
	roscpp
    rospy
	cairo_lfd
	message_generation
)

add_service_files(
	FILES
	PerformDemonstration.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	rospy
	cairo_lfd
)
