cmake_minimum_required(VERSION 3.8)

project(nftm_tester)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS nftm_tester/ego_centric_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ego_centric_map)

install(PROGRAMS nftm_tester/ground_height_grid.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ground_height_grid)

install(PROGRAMS nftm_tester/livox_tester.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME livox_tester)

install(PROGRAMS nftm_tester/realtime_foot_force.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME realtime_foot_force)

install(PROGRAMS nftm_tester/realtime_foot_z.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME realtime_foot_z)

install(PROGRAMS nftm_tester/plot_foot_force.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_foot_force)

ament_package()