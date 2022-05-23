cmake_minimum_required(VERSION 3.1)
project(mkecli_examples)

# Only the most crucial flags here (see libmkeclient's cmake for optimal flags)
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options("-std=c++11")
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE 
      "${CMAKE_CXX_FLAGS_RELEASE} /O2 /fp:fast /MT /DNDEBUG")

    set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif()

# This block is not needed if the libmkecli has been installed via `make install`
# Find mkeclient headers dir if it is not set by parent cmake
if (NOT MKECLI_INCLUDE_DIRS)
  find_path(
    MKECLI_INCLUDE_DIRS
    NAMES mke/cli/client.h
    HINTS ../
    PATH_SUFFIXES include)
endif()

# Find MkE Api headers dir if it is not set by parent cmake
if (NOT MKEAPI_INCLUDE_DIRS)
  find_path(
    MKEAPI_INCLUDE_DIRS
    NAMES mkeapi.h
    HINTS ../../../../../
    PATH_SUFFIXES 
      mkeapi/include)
endif()

# Find precompiled mkeclient library if it is not set by parent cmake
if (NOT LIBMKECLIENT)
  find_library(
    LIBMKECLIENT
    NAMES libmkeclient.a # Ubuntu
             mkeclient.lib # Win
    HINTS ../../build/src # Ubuntu
          ../../build/src/Release # Win
    PATH_SUFFIXES lib)
endif()

if (NOT LIBMKECLIENT)
  message(FATAL_ERROR "Cannot find the mkeclient library! "
                      "Compile it from `mkeclient` directory first.")
endif()

# Threads
find_package(Threads REQUIRED)

# These are needed for standalone build on Win
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
add_definitions(-DBOOST_FILESYSTEM_NO_LIB)
add_definitions(-DBOOST_SYSTEM_NO_LIB)

# Info prints only
message("The value of MKECLI_INCLUDE_DIRS is ${MKECLI_INCLUDE_DIRS}")
message("The value of MKEAPI_INCLUDE_DIRS is ${MKEAPI_INCLUDE_DIRS}")
message("The value of LIBMKECLIENT is ${LIBMKECLIENT}")
message("The value of Boost_INCLUDE_DIRS is ${Boost_INCLUDE_DIRS}")

# Demos
set(BINNAME0 "demo_device_discovery")
add_executable(${BINNAME0} demo_device_discovery.cpp)
target_link_libraries(${BINNAME0} ${LIBMKECLIENT}) 
target_link_libraries(${BINNAME0} Threads::Threads)
target_include_directories(${BINNAME0} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_include_directories(${BINNAME0} PRIVATE ${MKEAPI_INCLUDE_DIRS})

set(BINNAME1 "demo_device_state")
add_executable(${BINNAME1} demo_device_state.cpp)
target_link_libraries(${BINNAME1} ${LIBMKECLIENT})
target_link_libraries(${BINNAME1} Threads::Threads)
target_include_directories(${BINNAME1} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_include_directories(${BINNAME1} PRIVATE ${MKEAPI_INCLUDE_DIRS})

set(BINNAME2 "demo_getframe_sync")
add_executable(${BINNAME2} demo_getframe_sync.cpp)
target_link_libraries(${BINNAME2} ${LIBMKECLIENT})
target_link_libraries(${BINNAME2} Threads::Threads)
target_include_directories(${BINNAME2} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_include_directories(${BINNAME2} PRIVATE ${MKEAPI_INCLUDE_DIRS})

set(BINNAME3 "demo_getframe_async")
add_executable(${BINNAME3} demo_getframe_async.cpp)
target_link_libraries(${BINNAME3} ${LIBMKECLIENT})
target_link_libraries(${BINNAME3} Threads::Threads)
target_include_directories(${BINNAME3} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_include_directories(${BINNAME3} PRIVATE ${MKEAPI_INCLUDE_DIRS})

set(BINNAME4 "demo_pushframes")
add_executable(${BINNAME4} demo_pushframes.cpp)
target_link_libraries(${BINNAME4} ${LIBMKECLIENT})
target_link_libraries(${BINNAME4} Threads::Threads)
target_include_directories(${BINNAME4} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_include_directories(${BINNAME4} PRIVATE ${MKEAPI_INCLUDE_DIRS})

set(BIN_TEST_LATENCY "test_latency")
add_executable(${BIN_TEST_LATENCY} test_latency.cpp)
target_link_libraries(${BIN_TEST_LATENCY} ${LIBMKECLIENT})
target_link_libraries(${BIN_TEST_LATENCY} Threads::Threads)
target_include_directories(${BIN_TEST_LATENCY} PRIVATE ${MKEAPI_INCLUDE_DIRS})
target_include_directories(${BIN_TEST_LATENCY} PRIVATE ${MKECLI_INCLUDE_DIRS})
