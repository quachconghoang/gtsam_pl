# Install headers
file(GLOB sam_headers "*.h")
install(FILES ${sam_headers} DESTINATION include/gtsam/sam)
