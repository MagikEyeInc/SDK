# MkE Client integration demos
# 2020--2021, Ravi Kiran Nagaraj, ravi@magik-eye.com

cmake_minimum_required(VERSION 3.10)

PROJECT(MKE_CUSTOM_CLIENT)

option(MKE_USE_PCL "Build with PCL support" OFF)
option(MKE_USE_OPEN3D "Build with Open3D support" OFF)
option(MKE_USE_OPENCV "Build with OpenCV support" OFF)
option(MKE_ENABLE_VISU "Enable Visualization" OFF)

if(MKE_USE_PCL)
elseif(MKE_USE_OPEN3D)
elseif(MKE_USE_OPENCV)
else()
  message(FATAL_ERROR "No framework specified (MKE_USE_PCL or MKE_USE_OPEN3D or MKE_USE_OPENCV)")
endif()

# Definitions
if(MKE_USE_PCL)
  add_definitions(-DUSE_PCL)
endif()

if(MKE_USE_OPEN3D)
  add_definitions(-DUSE_OPEN3D)
endif()

if(MKE_USE_OPENCV)
  add_definitions(-DUSE_OPENCV)
endif()

if(MKE_ENABLE_VISU)
  add_definitions(-DENABLE_VISU)
endif()

# Packages
if(MKE_USE_PCL) # Tested with version 1.10.0 
  find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
endif()

if(MKE_USE_OPEN3D) # Tested with version 0.12.0
    if(WIN32)
        find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
    else()
        find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/cmake)
        find_package(Eigen3 3.3 REQUIRED NO_MODULE)
        find_package(fmt REQUIRED)
        find_package(glfw3 REQUIRED)
    endif()
    if(NOT Open3D_FOUND)
        message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
    endif()
endif()

if(MKE_USE_OPENCV) # Tested with version 4.2.0
  find_package(OpenCV REQUIRED)
endif()

find_package(Threads REQUIRED)

# Find MkE Api headers dir
find_path(
  MKEAPI_INCLUDE_DIRS
  NAMES mkeapi.h
  HINTS ../../../../../
  PATH_SUFFIXES 
    mkeapi/include)

# Find mkeclient headers dir 
find_path(
  MKECLI_INCLUDE_DIRS
  NAMES mke/cli/client.h
  HINTS ../
  PATH_SUFFIXES include) 

# Find precompiled mkeclient library
find_library(
  LIBMKECLIENT
  NAMES libmkeclient.a # Ubuntu
            mkeclient.lib # Win
  HINTS ../../build/src # Ubuntu
        ../../build/src/Release # Win
  PATH_SUFFIXES lib)

# Info prints only
message("The value of MKEAPI_INCLUDE_DIRS is ${MKEAPI_INCLUDE_DIRS}")
message("The value of MKECLI_INCLUDE_DIRS is ${MKECLI_INCLUDE_DIRS}")
message("The value of LIBMKECLIENT is ${LIBMKECLIENT}")
message("The value of Boost_INCLUDE_DIRS is ${Boost_INCLUDE_DIRS}")

if (NOT LIBMKECLIENT)
  message(FATAL_ERROR "Cannot find the mkeclient library! "
                      "Compile it from `mkeclient` directory first.")
endif() 

# Includes 
include_directories(${MKEAPI_INCLUDE_DIRS})
include_directories(${MKECLI_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})

if(MKE_USE_PCL)
  include_directories(${PCL_INCLUDE_DIRS})
endif()

if(MKE_USE_OPEN3D)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

if(MKE_USE_OPENCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

if(MKE_USE_PCL)   
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

# Targets 
if(MKE_USE_PCL)
  add_executable(client_pcl_demo client_pcl_demo.cpp)
  target_link_libraries(client_pcl_demo ${LIBMKECLIENT} Threads::Threads ${PCL_LIBRARIES})
endif()

if(MKE_USE_OPEN3D)
  add_executable(client_open3d_demo client_open3d_demo.cpp) 
  target_link_libraries(client_open3d_demo ${LIBMKECLIENT} Threads::Threads Open3D::Open3D)
endif()

if(MKE_USE_OPENCV)
  add_executable(client_opencv_demo client_opencv_demo.cpp) 
  target_link_libraries(client_opencv_demo ${LIBMKECLIENT} Threads::Threads ${OpenCV_LIBS})
endif()
