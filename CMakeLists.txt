cmake_minimum_required(VERSION 3.10)
set(RAISIM_VERSION 1.1.0)
project(raisim VERSION ${RAISIM_VERSION} LANGUAGES CXX)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Include CMake helper modules
include(GNUInstallDirs)

find_package(Eigen3)

# Add some ROS
find_package(catkin REQUIRED
 roscpp
 trajectory_msgs
 COMPONENTS
 )


# Build options
option(RAISIM_EXAMPLE "Build example applications." FALSE)
option(RAISIM_MATLAB "Build a matlab wrapper." FALSE)
option(RAISIM_PY "Build a python wrapper." FALSE)
option(RAISIM_DOC "Build a raisim documentation. You can find the latest doc in raisim.com." FALSE)


set(RAISIM_OS linux)
set(RAISIM_BIN_DIR ${RAISIM_OS})


catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/include
  LIBRARIES raisim
  CATKIN_DEPENDS roscpp
)
include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/include
  ${catkin_INCLUDE_DIRS}
  )

set (_IMPORT_PREFIX ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR})
# Create imported target raisim::raisimZ
add_library(raisim::raisimZ SHARED IMPORTED)

set_target_properties(raisim::raisimZ PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
)

# Create imported target raisim::raisimPng
add_library(raisim::raisimPng SHARED IMPORTED)

set_target_properties(raisim::raisimPng PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "raisim::raisimZ"
)

# Create imported target raisim::raisimMine
add_library(raisim::raisimMine SHARED IMPORTED)

set_target_properties(raisim::raisimMine PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
)

# Create imported target raisim::raisimODE
add_library(raisim::raisimODE SHARED IMPORTED)

set_target_properties(raisim::raisimODE PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "dIDEDOUBLE;dIDEDOUBLE"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
)

# Create imported target raisim::raisim
add_library(raisim::raisim SHARED IMPORTED)

set_target_properties(raisim::raisim PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "raisim::raisimPng;raisim::raisimODE;raisim::raisimMine;${catkin_LIBRARIES}"
)

# Install raisim lib
file(
  COPY
  ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/
  DESTINATION
  ${CATKIN_DEVEL_PREFIX}
)


if(RAISIM_ALL)
    set(RAISIM_MATLAB TRUE)
    set(RAISIM_PY TRUE)
    set(RAISIM_DOC TRUE)
endif()


if(RAISIM_MATLAB)
#    add_subdirectory(raisimMatlab)
endif()

if(RAISIM_PY)
#    add_subdirectory(thirdParty/pybind11)
#    add_subdirectory(raisimPy)
endif()

if(RAISIM_EXAMPLE)
    add_subdirectory(examples)
endif()

if(RAISIM_DOC)
    add_subdirectory(docs)
endif()

# EOF
