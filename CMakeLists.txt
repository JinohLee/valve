project(jinoh_arash_valve)

################################################################
#    Coman project header. Don't touch unless you know how     # 
################################################################
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

##################################################################
#    Project files.  Adjust to fit your project.                 #
##################################################################
SET(ARMADILLO_LIB_USER_PATH "/usr/lib" CACHE STRING "folder where armadillo libs are installed")
SET(ARMADILLO_INCLUDE_USER_PATH "/usr/include" CACHE STRING "folder where armadillo libs are installed")

include_directories(include external/include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
# add required linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

include_directories(${ARMADILLO_INCLUDE_USER_PATH})
link_directories(${ARMADILLO_LIB_USER_PATH})

link_directories("/usr/lib/libblas/")
link_directories("/usr/lib/lapack/")

set(SOURCES
src/main_yarp.cpp
src/valve_yarp.cpp
include/valve_yarp.h
src/yarp_interface.cpp
include/yarp_interface.h
include/interface.h
src/mySkew.cpp
include/mySkew.h
src/Traj_gen.cpp
include/Traj_gen.h
src/orientation_ctrl.cpp
include/orientation_ctrl.h
src/ManipulationVars.cpp
include/ManipulationVars.h
src/ValveModule.cpp
include/ValveModule.h
src/robolli_legacy.cpp
include/robolli_legacy.h
)


set(LIBRARIES
m
dl
stdc++
pthread
rt
yaml-cpp
armadillo
lapack
blas
${YARP_LIBRARIES}
icubmod
${ICUB_LIBRARIES}
)

add_executable(jinoh_arash_valve ${SOURCES})
add_custom_command(TARGET jinoh_arash_valve POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy "${PROJECT_SOURCE_DIR}/config.yaml" "config.yaml")
target_link_libraries(jinoh_arash_valve ${LIBRARIES})
install(TARGETS jinoh_arash_valve DESTINATION bin)
