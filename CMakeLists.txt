project(coman_basic_example)

################################################################
#    Coman project header. Don't touch unless you know how     # 
################################################################
cmake_minimum_required(VERSION 2.8)

#set (ENV{COMAN_ROOT} "/usr/local/src/COMAN_shared")
set (ENV{COMAN_ROOT} "/home/jinoh/CoMan/COMAN_shared")


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF("$ENV{COMAN_ROOT}" STREQUAL "")
  message(FATAL_ERROR "!!! No COMAN_ROOT environment variable. !!!\n"
          "Try: >export COMAN_ROOT=/location/of/coman/libraries\n" $ENV{COMAN_ROOT})
ELSE()
  message("Install root is: " $ENV{COMAN_ROOT})
  set(COMAN_ROOT $ENV{COMAN_ROOT}) 
ENDIF()
 
#Install into source folder unless user sets something different
SET(MY_INSTALL_PREFIX ${COMAN_ROOT} CACHE PATH "Prefix prepended to install directories")
SET(CMAKE_INSTALL_PREFIX "${MY_INSTALL_PREFIX}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

##################################################################
#    Project files.  Adjust to fit your project.                 #
##################################################################
#---CoMan_Shared
include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")
#---Armadillo
include_directories("/usr/include")
link_directories("/usr/lib")
#---Open-blas
link_directories("/usr/lib/")
#link_directories("/usr/lib/libblas/")
#---Lapack
link_directories("/usr/lib/")
#link_directories("/usr/lib/lapack/")

add_definitions(" -std=c++0x -Duse_namespace")

set(SOURCES
Boards_ctrl_basic.cpp
test_main.cpp
mySkew.cpp
Traj_gen.cpp
orientation_ctrl.cpp
)


set(LIBRARIES
m
dl
stdc++
pthread
rt
coman_os
robolli
coman_microstrain
yaml-cpp
armadillo
lapack
blas
)

add_executable(coman_basic_example ${SOURCES})

target_link_libraries(coman_basic_example ${LIBRARIES})
install(TARGETS coman_basic_example DESTINATION bin)
