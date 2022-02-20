cmake_minimum_required(VERSION 3.10)

# Language
enable_language( CXX )

# Compiler Settings
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
# find_package(Open3D REQUIRED)

find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
  include_directories( ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  # target_link_libraries( project ${PCL_LIBRARIES} )
endif()

add_executable(test_projection  test_projection.cpp)
target_link_libraries(test_projection gtsam ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#target_link_libraries(test_projection gtsam Open3D::Open3D ${Boost_LIBRARIES})
