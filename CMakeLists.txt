project(jinoh_arash_valve)

################################################################
#    Coman project header. Don't touch unless you know how     # 
################################################################
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

# COMAN_ROOT should be set in your bash script, or it can be set from the option
#set (ENV{COMAN_ROOT} "/home/coman/AJ/COMAN_shared")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF("$ENV{COMAN_ROOT}" STREQUAL "")
  message(WARNING "!!! No COMAN_ROOT environment variable. !!!\n"
          "Try: >export COMAN_ROOT=/location/of/coman/libraries\n"
          "Otherwise, run ccmake to set manually the COMAN_ROOT variable")
	SET(COMAN_ROOT "/home/coman/COMAN_shared" CACHE STRING "folder where robolli is locate, e.g. /home/coman/COMAN_shared")

ELSE()
  message("Install root is: " $ENV{COMAN_ROOT})
  set(COMAN_ROOT $ENV{COMAN_ROOT}) 
ENDIF()
 
#Install into source folder unless user sets something different
#SET(MY_INSTALL_PREFIX ${COMAN_ROOT} CACHE PATH "Prefix prepended to install directories")
#SET(CMAKE_INSTALL_PREFIX "${MY_INSTALL_PREFIX}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

##################################################################
#    Project files.  Adjust to fit your project.                 #
##################################################################
SET(ARMADILLO_LIB_USER_PATH "external/armadillo/usr/lib" CACHE STRING "folder where armadillo libs are installed")
SET(ARMADILLO_INCLUDE_USER_PATH "external/armadillo/usr/include" CACHE STRING "folder where armadillo libs are installed")



include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

include_directories(${ARMADILLO_INCLUDE_USER_PATH})
link_directories(${ARMADILLO_LIB_USER_PATH})

link_directories("/usr/lib/libblas/")
link_directories("/usr/lib/lapack/")


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

add_executable(jinoh_arash_valve ${SOURCES})
add_custom_command(TARGET jinoh_arash_valve POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy "${PROJECT_SOURCE_DIR}/config.yaml" config.yaml)
target_link_libraries(jinoh_arash_valve ${LIBRARIES})
install(TARGETS jinoh_arash_valve DESTINATION bin)
