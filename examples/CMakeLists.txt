#set(Open3D_DIR /home/hoangqc/open3d_install/lib/cmake/)
set (excluded_examples
    Pose2*.cpp
    Pose3*.cpp
    ISAM2Example_SmartFactor.cpp
    VisualISAM*.cpp
    SFMExampleExpressions*.cpp
    SFMExample_SmartFactor*.cpp
    StereoVOExample*.cpp
    CameraResectioning.cpp

)

# Install Open3D at /usr/local/ ???
find_package(Open3D REQUIRED)
gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY};Open3D::Open3D")
