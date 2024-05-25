# Set the minimum required version of CMake for this project
cmake_minimum_required(VERSION 3.0.2)

# Define the project name
project(sensor_fusion)

# Uncomment the following line to compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries
# If COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

# Declare a catkin package
catkin_package()

# Include directories for catkin
include_directories(${catkin_INCLUDE_DIRS})

# Include directories for classes
include_directories(include/classes)

# Add a library for sensor fusion
add_library(sensor_fusion_lib
  include/classes/Localization.cpp
  include/classes/Particle_Filter.cpp
  include/classes/Motion_Model.cpp
  include/classes/Communication.cpp
)

############################################################ CMakelist for main node ############################################################
# Include directories for structs
include_directories(include/structs)

# Add an executable for the main program
add_executable(main src/main.cpp)

# Link the main program with the sensor fusion library and catkin libraries
target_link_libraries(main sensor_fusion_lib ${catkin_LIBRARIES})

# Add dependencies for the main program
add_dependencies(main ${sensor_fusion_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############################################################ CMakelist for my_test node ############################################################

# Add an executable for the test program
add_executable(my_test src/my_test.cpp)

# Link the test program with the sensor fusion library and catkin libraries
target_link_libraries(my_test sensor_fusion_lib ${catkin_LIBRARIES})

# Add dependencies for the test program
add_dependencies(my_test ${sensor_fusion_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})