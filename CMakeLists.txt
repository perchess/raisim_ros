cmake_minimum_required(VERSION 3.10)
set(RAISIM_VERSION 1.1.0)
project(raisim VERSION ${RAISIM_VERSION} LANGUAGES CXX)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Include CMake helper modules
include(GNUInstallDirs)

# Add some ROS
find_package(catkin REQUIRED
 roscpp
 trajectory_msgs
 COMPONENTS
 )


# ROS

# Build options
option(RAISIM_EXAMPLE "Build example applications." TRUE)
option(RAISIM_MATLAB "Build a matlab wrapper." FALSE)
option(RAISIM_PY "Build a python wrapper." FALSE)
option(RAISIM_DOC "Build a raisim documentation. You can find the latest doc in raisim.com." FALSE)

#if (UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL AND NOT WIN32)
    set(RAISIM_OS linux)
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS})
#elseif(APPLE)
#    execute_process(COMMAND sysctl -q hw.optional.arm64
#            OUTPUT_VARIABLE _sysctl_stdout
#            ERROR_VARIABLE _sysctl_stderr
#            RESULT_VARIABLE _sysctl_result
#            )
#    if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
#        set(RAISIM_OS m1)
#    else()
#        set(RAISIM_OS mac)
#    endif()
#    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS})
#elseif(WIN32)
#    set(RAISIM_OS win32)
#    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS}/mt_release)
#    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS}/mt_debug)
#    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/Eigen3)
#endif()

#==
# Package installation
#==
#if(WIN32)
#    set(RAISIM_BIN_DIR ${RAISIM_OS}/mt_release)
#    install(
#        DIRECTORY
#        ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/bin
#        DESTINATION
#        ${CMAKE_INSTALL_PREFIX}
#    )
#else()
    set(RAISIM_BIN_DIR ${RAISIM_OS})
#endif()


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



# install(
#         DIRECTORY
#         ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/include
#         DESTINATION
#         ${CMAKE_INSTALL_PREFIX}
# )
#
# install(
#         DIRECTORY
#         ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/
#         DESTINATION
#         /home/den/c_workspaces/raisim_pure_ws/devel/.private/raisim
# )
file(
  COPY
  ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/
  DESTINATION
  /home/den/c_workspaces/raisim_pure_ws/devel/.private/raisim
)

message ("FROM " ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/)
message ("TO " /home/den/c_workspaces/raisim_pure_ws/devel/.private/raisim)
# install(TARGETS raisim
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Установить библиотеку в cmake install директорию
#install(FILES ${qpOASES_CXX_LIBRARY}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

install(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}")

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

if(RAISIM_DOC)
    add_subdirectory(docs)
endif()

# EOF
