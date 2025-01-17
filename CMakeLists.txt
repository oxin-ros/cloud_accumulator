cmake_minimum_required(VERSION 3.10.2)

# Don't set PROJECT_VERSION to empty string when no VERSION is given to project() command.
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 OLD)
endif()

project(cloud_accumulator)
include(cmake/project_version.cmake)

# Use C++ 17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# - Configuration for code optimization -
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

include(cmake/optimization_flags.cmake)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf2
    tf2_msgs
    tf2_ros
    std_srvs
)


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rc_slam_viewer
#  CATKIN_DEPENDS rc_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cc )
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

######################################
## Define information for packaging ##
######################################
# optionally set description and maintainer (if not set will use defaults from package_debian.cmake)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Viewer for the point clouds.")

# optionally set package maintainer (otherwise defaults to Roboception <info@roboception.de>)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Felix Endres <felix.endres@roboception.de>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-rc-visard-driver, ros-$ENV{ROS_DISTRO}-pcl-ros")
include(cmake/package_debian.cmake)
