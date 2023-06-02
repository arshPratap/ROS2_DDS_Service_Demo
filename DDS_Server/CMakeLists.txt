# --------------------------------------------------------------------------- #
# Build.

add_executable(dds_server_demo Server.c)
target_include_directories(dds_server_demo
  PRIVATE
  ${microcdr_INCLUDE_DIR}
  ${microxrcedds_client_INCLUDE_DIR}  
)
target_link_directories(dds_server_demo
  PRIVATE
  ${microcdr_LIB_DIR}
  ${microxrcedds_client_LIB_DIR}  
)
target_link_libraries(dds_server_demo
  microcdr
  microxrcedds_client
)

# --------------------------------------------------------------------------- #
#  Install.

install(TARGETS
  dds_server_demo
  RUNTIME DESTINATION bin/
)

install(FILES
  fastdds_server__addtwoints.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# --------------------------------------------------------------------------- #
