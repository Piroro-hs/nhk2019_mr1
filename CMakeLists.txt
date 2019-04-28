cmake_minimum_required(VERSION 2.8.3)
project(nhk2019_mr1)
find_package(catkin REQUIRED)
catkin_package()
add_custom_target(
  npm ALL
  COMMAND cd "${CMAKE_CURRENT_SOURCE_DIR}/src/main" && npm install
  COMMAND cd "${CMAKE_CURRENT_SOURCE_DIR}/src/omni" && npm install
)
