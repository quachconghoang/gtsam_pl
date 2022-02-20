cmake_minimum_required(VERSION 3.10)
# Language
enable_language( CXX )
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

project(appSLAM CXX)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# Boost - same requirement as gtsam
find_package(Boost 1.65 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
add_executable(test_projection LinesProjectionFactor.h test_projection.cpp)
target_link_libraries(test_projection gtsam ${Boost_LIBRARIES})

#find_package( PCL 1.8 REQUIRED )
#if( PCL_FOUND )
#  include_directories( ${PCL_INCLUDE_DIRS} )
#  add_definitions( ${PCL_DEFINITIONS} )
#  # For Use Not PreCompiled Features
#  add_definitions( -DPCL_NO_PRECOMPILE )
#  link_directories( ${PCL_LIBRARY_DIRS} )
#endif()
#target_link_libraries(test_projection gtsam ${Boost_LIBRARIES} ${PCL_LIBRARIES})
