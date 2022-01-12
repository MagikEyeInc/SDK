# This is libmkeclient: C++ library implementing MKE API client interface
# 2016--2021, Jan Heller, jan@magik-eye.com

cmake_minimum_required(VERSION 3.0)
project(mkeclient C CXX)

option(MKECLI_DEMO "Build Demo programs" OFF)
option(MKECLI_TESTER "Build libmkecli Tester" OFF)

set(MKECLI_VERSION_MAJOR 0)
set(MKECLI_VERSION_MINOR 1)
set(MKECLI_VERSION_PATCH 0)
set(MKECLI_VERSION
    ${MKECLI_VERSION_MAJOR}.${MKECLI_VERSION_MINOR}.${MKECLI_VERSION_PATCH})

add_definitions(-DMKECLI_VERSION_MAJOR="${MKECLI_VERSION_MAJOR}")
add_definitions(-DMKECLI_VERSION_MINOR="${MKECLI_VERSION_MINOR}")
add_definitions(-DMKECLI_VERSION_PATCH="${MKECLI_VERSION_PATCH}")
add_definitions(-DMKECLI_VERSION="${MKECLI_VERSION}")

set(LIBMKECLIENT "mkeclient")
set(LIBMKECLIENT ${LIBMKECLIENT} PARENT_SCOPE)

if(NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE
      RELEASE
      CACHE STRING "Choose the type of build, options are: None Debug Release"
            FORCE)
endif()

# #############################################################################
# CXX Flags

if(MSVC)
    #set(CMAKE_CXX_FLAGS_RELEASE 
    #  "${CMAKE_CXX_FLAGS_RELEASE} /O2 /fp:fast /MT /DNDEBUG")

    #set(CMAKE_CXX_FLAGS_DEBUG
    #  "${CMAKE_CXX_FLAGS_DEBUG} /MT")
      
else()
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wall -Wextra \
        -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter\
            -Wno-missing-field-initializers -fpic")

    set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3")
    set(CMAKE_CXX_FLAGS_DEBUG "-pg -ggdb -Og")
endif()

# #############################################################################
# MkE API headers

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
find_package(MKEAPI)

# #############################################################################
# Boost

set(Boost_NO_BOOST_CMAKE to ON)
find_package(Boost REQUIRED)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
add_definitions(-DBOOST_FILESYSTEM_NO_LIB)
add_definitions(-DBOOST_SYSTEM_NO_LIB)

# #############################################################################
# Subdirectories
add_subdirectory(src)
