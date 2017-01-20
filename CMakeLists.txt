#########################################################
#                                                       #
#    Control Simulators                                 #
#    - Roberto Capobianco  <capobianco@dis.uniroma1.it> #
#    - Arun Venkatraman    <arunvenk@cs.cmu.edu>        #
#                                                       #
#    Created:  March 2016                               #
#                                                       #
#########################################################


################## Header  definitions ##################

cmake_minimum_required(VERSION 2.7)
project(control_simulators)

# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set(EXTERNALS_DIR ${PROJECT_SOURCE_DIR}/externals)

#########################################################

############# External Projects Collection ##############

include(ExternalProject)

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

ExternalProject_Add(boost_numpy_external
    GIT_REPOSITORY https://github.com/personalrobotics/Boost.NumPy
    PREFIX ${PROJECT_SOURCE_DIR}/externals
    INSTALL_DIR ${PROJECT_SOURCE_DIR}/externals
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/externals
    -DPYTHON_LIBRARIES=${PYTHON_LIBRARIES}
    -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIRS}
    -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
)

ExternalProject_Add(boost_numpy_eigen_external
    GIT_REPOSITORY https://github.com/personalrobotics/Boost.NumPy_Eigen.git
    PREFIX ${EXTERNALS_DIR}
    INSTALL_DIR ${EXTERNALS_DIR}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNALS_DIR}
    -DMODULE_INSTALL_PREFIX=${LIBRARY_OUTPUT_PATH}
    -DPYTHON_LIBRARIES=${PYTHON_LIBRARIES}
    -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIRS}
    -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
)
add_dependencies(boost_numpy_eigen_external boost_numpy_external)

#########################################################

############### Configuration  Generation ###############

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS python random REQUIRED)
find_package(OpenMP)

# if (OpenCV_FOUND)
#	message(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS} (found version \"${OpenCV_VERSION}\")")
# endif (OpenCV_FOUND)

if (CMAKE_BUILD_TYPE)
  string(TOUPPER ${CMAKE_BUILD_TYPE} _build_type)
  set(CMAKE_BUILD_TYPE ${_build_type} CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE)

# Build type must be either DEBUG or RELEASE.
if (CMAKE_BUILD_TYPE STREQUAL "")
  message("No build type specified: forcing to RELEASE.")
  set(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE STREQUAL "")

# Linux bitness type.
if (LINUX_BITNESS)
  string(TOUPPER ${LINUX_BITNESS} _linux_bitness)
  set(LINUX_BITNESS ${_linux_bitness} CACHE STRING "" FORCE)
else (LINUX_BITNESS)
  set(LINUX_BITNESS "" CACHE STRING "" FORCE)
endif (LINUX_BITNESS)

# Linux bitness type must be either 32-BIT or 64-BIT.
if (LINUX_BITNESS STREQUAL "")
  EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
  
  if (${ARCHITECTURE} STREQUAL "x86_64")
    message("No linux bitness type specified. Detected 64-bit architecture, forcing to 64-bit bitness compilation.")
    set(LINUX_BITNESS "64-BIT" CACHE STRING "" FORCE)
  elseif (${ARCHITECTURE} STREQUAL "i386")
    message("No linux bitness type specified. Detected 32-bit architecture, forcing to 32-bit bitness compilation.")
    set(LINUX_BITNESS "32-BIT" CACHE STRING "" FORCE)
  else (${ARCHITECTURE} STREQUAL "i386")
    message(FATAL_ERROR "Unknown architecture found. The library cannot be compiled under this architecture yet.")
  endif (${ARCHITECTURE} STREQUAL "x86_64")
endif (LINUX_BITNESS STREQUAL "")

#########################################################

##################### Flag  Setting #####################

set(ControlSim_CXX_FLAGS "")

if (OPENMP_FOUND)
  set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_USE_OPENMP_")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OPENMP_FOUND)

if (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} -ggdb -frounding-math")
elseif (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
  set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} -O3")
else (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  message(FATAL_ERROR "Invalid build type specified: doing nothing.")
endif (CMAKE_BUILD_TYPE STREQUAL "DEBUG")

if (LINUX_BITNESS STREQUAL "32-BIT")
  set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} -m32")
elseif (LINUX_BITNESS STREQUAL "64-BIT")
  set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} -m64")
else (LINUX_BITNESS STREQUAL "32-BIT")
  message(FATAL_ERROR "Invalid linux bitness type specified.")
endif (LINUX_BITNESS STREQUAL "32-BIT")

set(ControlSim_CXX_FLAGS "${ControlSim_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${ControlSim_CXX_FLAGS} -Wall")

#########################################################

#################### Code  Compiling ####################

file(GLOB_RECURSE ControlSim_src "src/control_simulators/*.cpp")
set(ControlSimTest_src "src/simulator_test")
set(ControlSimPython_src "src/python")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${EXTERNALS_DIR}/include)
link_directories(${EXTERNALS_DIR}/lib)

add_library(control_simulators SHARED ${ControlSim_src})
target_link_libraries(control_simulators pthread ${Boost_LIBRARIES} )

add_executable(pendulum_sim_test "${ControlSimTest_src}/pendulum_sim_test.cpp")
target_link_libraries(pendulum_sim_test control_simulators)

add_executable(cartpole_sim_test "${ControlSimTest_src}/cartpole_sim_test.cpp")
target_link_libraries(cartpole_sim_test control_simulators)

function(SetPythonLibParams targetname)
    set_target_properties(${targetname} PROPERTIES PREFIX "" SUFFIX ".so")
    add_dependencies(${targetname} boost_numpy_external boost_numpy_eigen_external)
    target_link_libraries(${targetname} control_simulators boost_python boost_numpy ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
endfunction()
# python bindings
add_library(python_simulable SHARED ${ControlSimPython_src}/python_simulable.cpp)
SetPythonLibParams(python_simulable) 

#########################################################

################# Software Installation #################

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})

# Headers.
install(DIRECTORY src/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/control_simulators FILES_MATCHING PATTERN "*.h")

# Shared objects.
# install(TARGETS lib/ControlSim LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/control_simulators)

# Binaries.
# install(TARGETS bin/ControlSim RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/control_simulators)

