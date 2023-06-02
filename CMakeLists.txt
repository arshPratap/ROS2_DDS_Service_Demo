cmake_minimum_required(VERSION 3.20)
project(ros2_dds_service_demo)

# --------------------------------------------------------------------------- #
# Dependencies.

find_package(microcdr)
find_package(microxrcedds_client)

# Debug.
# message("microcdr_FOUND: ${microcdr_FOUND}")
# message("microcdr_INCLUDE_DIR: ${microcdr_INCLUDE_DIR}")
# message("microcdr_LIB_DIR: ${microcdr_LIB_DIR}")
# message("microxrcedds_client_FOUND: ${microxrcedds_client_FOUND}")
# message("microxrcedds_client_INCLUDE_DIR: ${microxrcedds_client_INCLUDE_DIR}")
# message("microxrcedds_client_LIB_DIR: ${microxrcedds_client_LIB_DIR}")

# --------------------------------------------------------------------------- #
# Subdirectories

add_subdirectory(DDS_Server)
add_subdirectory(ROS2_Server)

# --------------------------------------------------------------------------- #
