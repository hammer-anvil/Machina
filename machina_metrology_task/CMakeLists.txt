cmake_minimum_required(VERSION 3.8)
project(machina_metrology_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters registration common io surface)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Add executables
add_executable(data_generator_calibration src/data_generator_calibration.cpp)
target_include_directories(data_generator_calibration PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(data_generator_calibration rclcpp Eigen3 geometry_msgs)

add_executable(data_generator_pcd src/data_generator_pcd.cpp)
target_include_directories(data_generator_pcd PUBLIC
  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
)
target_link_libraries(data_generator_pcd ${PCL_LIBRARIES})
ament_target_dependencies(data_generator_pcd rclcpp PCL sensor_msgs pcl_conversions)

add_executable(hand_eye_calibration src/hand_eye_calibration.cpp)
target_include_directories(hand_eye_calibration PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(hand_eye_calibration 
  rclcpp 
  Eigen3 
  geometry_msgs 
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
)

add_executable(pcd_filter_align src/pcd_filter_align.cpp)
target_include_directories(pcd_filter_align PUBLIC
  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd_filter_align ${PCL_LIBRARIES} ${PCL_SURFACE_LIBRARIES})
ament_target_dependencies(pcd_filter_align rclcpp PCL sensor_msgs pcl_conversions tf2 tf2_ros std_srvs)

# Install targets
install(TARGETS
  data_generator_calibration
  data_generator_pcd
  hand_eye_calibration
  pcd_filter_align
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Create and install data directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/pcd_data
)

ament_package()