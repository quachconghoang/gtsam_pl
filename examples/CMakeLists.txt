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

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}")
