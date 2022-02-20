set (excluded_examples
    Pose2*.cpp
    Pose3*.cpp
    ISAM2Example_SmartFactor.cpp
    VisualISAM*.cpp
    SFMExampleExpressions*.cpp
    SFMExample_SmartFactor*.cpp
    StereoVOExample*.cpp
    CameraResectioning.cpp SFMTrace.cpp
)

# Install Open3D at /usr/local/ ???
gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}")
# find_package(Open3D REQUIRED)
#add_executable(SFMTrace SFMTrace.cpp)
#target_link_libraries(SFMTrace "Open3D::Open3D;gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}")
#target_link_libraries(SFMTrace PRIVATE Open3D::Open3D)
#message(STATUS "Open3d -- " Open3D::Open3D)