# Set the minimum required version of CMake for this project
cmake_minimum_required(VERSION 3.0.2)

# Define the project name
project(sensor_fusion)

# Uncomment the following line to compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  roslib
)

find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
# message(STATUS "Python3 NumPy include dirs: ${Python3_NumPy_INCLUDE_DIRS}")

# include(FetchContent)

# FetchContent_Declare(matplotlib
#     GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
#     GIT_TAG f23347fca25219d1c42cbb91608b5556814bf572
# )
# FetchContent_MakeAvailable(matplotlib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${matplotlib_SOURCE_DIR}
  include
  include/classes  # Include the directory where your headers are located
)

# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/Sensor_Model.cfg
  cfg/Motion_Model.cfg
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_fusion_lib
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)

# Add a library for sensor fusion
add_library(sensor_fusion_lib
  include/classes/Particle_Filter.cpp
  include/classes/Motion_Model.cpp
  include/classes/Communication.cpp
  include/classes/Visualizer.cpp
  include/classes/Sensor_Model.cpp
)

# Link the sensor fusion library with the catkin libraries
target_link_libraries(sensor_fusion_lib ${catkin_LIBRARIES})

############################################################
# CMakelist for main node
############################################################

# Add an executable for the main program
add_executable(main src/main.cpp)

# Link the main program with the sensor fusion library and catkin libraries
target_link_libraries(main sensor_fusion_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the main program
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############################################################
# CMakelist for my_test node
############################################################

# Add an executable for the test program
add_executable(my_test src/my_test.cpp)

# Link the test program mit der sensor fusion library und catkin libraries
target_link_libraries(my_test sensor_fusion_lib ${catkin_LIBRARIES} Python3::Python Python3::NumPy)

# Add dependencies for the test program
add_dependencies(my_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############################################################
# CMakelist for plot_histogram node
############################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/plot_histogram.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)