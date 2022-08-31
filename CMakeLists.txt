cmake_minimum_required(VERSION 3.5)
project(g_icp_ros)

list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
    include
    "/usr/include/eigen3"
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/g_icp_ros.cpp src/g_icp_ros_main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs tf2_geometry_msgs pcl_conversions tf2 tf2_ros)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

add_executable(pcl_publisher_node src/pcl_publisher.cpp src/pcl_publisher_main.cpp)
ament_target_dependencies(pcl_publisher_node rclcpp sensor_msgs tf2_geometry_msgs pcl_conversions)
target_link_libraries(pcl_publisher_node ${PCL_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

install(TARGETS
  ${PROJECT_NAME}_node pcl_publisher_node
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
