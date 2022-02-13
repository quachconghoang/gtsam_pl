# Install headers
file(GLOB sfm_headers "*.h")
install(FILES ${sfm_headers} DESTINATION include/gtsam/sfm)