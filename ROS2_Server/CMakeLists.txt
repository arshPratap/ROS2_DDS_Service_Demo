cmake_minimum_required(VERSION 3.20)
project(ros2_server_demo)

# --------------------------------------------------------------------------- #
# Dependencies.

find_package(microcdr)
find_package(microxrcedds_client)

# --------------------------------------------------------------------------- #
# Build.

add_executable(ros2_server_demo Client.c)
target_include_directories(ros2_server_demo
  PRIVATE
  ${microcdr_INCLUDE_DIR}
  ${microxrcedds_client_INCLUDE_DIR}  
)
target_link_directories(ros2_server_demo
  PRIVATE
  ${microcdr_LIB_DIR}
  ${microxrcedds_client_LIB_DIR}  
)
target_link_libraries(ros2_server_demo
  microcdr
  microxrcedds_client
)

# --------------------------------------------------------------------------- #
#  Install.

install(TARGETS
  ros2_server_demo
  RUNTIME DESTINATION bin/
)

install(FILES
  ros2_server__addtwoints.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# --------------------------------------------------------------------------- #
