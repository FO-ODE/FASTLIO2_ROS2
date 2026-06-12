cmake_minimum_required(VERSION 3.8)

project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae launch meshes rviz urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fastlio_lidar_to_base_tf.py
    scripts/livox_custom_to_pointcloud2.py
    scripts/lowstate_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
