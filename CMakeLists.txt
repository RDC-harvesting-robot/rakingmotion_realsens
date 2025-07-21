cmake_minimum_required(VERSION 3.22)
project(rakingmotion_realsens)

# --- 必要なパッケージの検索 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2 REQUIRED)


# --- realsense_closest_node 実行ファイル追加 ---
add_executable(realsense_closest_node src/realsense_closest_node.cpp)
ament_target_dependencies(realsense_closest_node 
  rclcpp
  geometry_msgs
  visualization_msgs
  sensor_msgs
  cv_bridge
  image_geometry
  message_filters
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(realsense_closest_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(realsense_closest_node
  ${OpenCV_LIBS}
  realsense2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# --- launch・config ディレクトリのインストール ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- 実行ファイルのインストール ---
install(TARGETS

  realsense_closest_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
