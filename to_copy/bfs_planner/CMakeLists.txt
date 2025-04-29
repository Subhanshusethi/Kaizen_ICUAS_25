cmake_minimum_required(VERSION 3.8)
project(bfs_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(bfs_connectivity_3d_map_node src/bfs_connectivity_3d_map_node.cpp)
add_executable(astar_pathfinder_3d_node src/astar_pathfinder_3d_node.cpp)
add_executable(save_wp src/save_wayp_astar.cpp)
add_executable(2d_astar src/2d_astar.cpp)
add_executable(bfs_slice src/bfs_slice.cpp)
add_executable(astar_slice src/astar_slice.cpp)
add_executable(uav_slice src/uavcontrol_slice.cpp)

ament_target_dependencies(
	bfs_connectivity_3d_map_node
	rclcpp
        nav_msgs
	sensor_msgs
	std_msgs
	geometry_msgs
	visualization_msgs
	octomap
	octomap_msgs
	)
ament_target_dependencies(
       	astar_pathfinder_3d_node
	rclcpp
	nav_msgs
	sensor_msgs
	std_msgs
	geometry_msgs
	visualization_msgs
	octomap
	octomap_msgs
	)
ament_target_dependencies(
        save_wp
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        visualization_msgs
        octomap
        octomap_msgs
        )
ament_target_dependencies(
        2d_astar
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        visualization_msgs
        octomap
        octomap_msgs
        )
ament_target_dependencies(
        bfs_slice
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        visualization_msgs
        octomap
        octomap_msgs
        )
ament_target_dependencies(
        astar_slice
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        visualization_msgs
        octomap
        octomap_msgs
        )
ament_target_dependencies(
        uav_slice
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        visualization_msgs
        octomap
        octomap_msgs
        )
# Install targets
install(TARGETS
	bfs_connectivity_3d_map_node
	astar_pathfinder_3d_node
	save_wp
	2d_astar
        bfs_slice
        astar_slice
        uav_slice
	DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY launch config
#	DESTINATION share/${PROJECT_NAME}
#      	)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
