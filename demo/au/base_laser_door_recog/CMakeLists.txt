cmake_minimum_required(VERSION 2.8.3)
project(base_laser_door_recog)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  sensor_msgs
)

catkin_package()

# include_directories(include ${PCL_INCLUDE_DIRS})
# link_libraries(${PCL_LIBERARIES})

# add_executable(random_sample_consensus src/random_sample_consensus.cpp)
# target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(line_detector src/line_detector.cpp)
target_link_libraries(line_detector ${catkin_LIBRARIES})