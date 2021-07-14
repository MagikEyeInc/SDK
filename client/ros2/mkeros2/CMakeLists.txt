cmake_minimum_required(VERSION 3.5)
project(mkeros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package (Threads REQUIRED)

# ##############################################################################
# Build ##
# ##############################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
find_package(MKECLI REQUIRED)

set(MKEROS2_VERSION_MAJOR 1)
set(MKEROS2_VERSION_MINOR 0)
set(MKEROS2_VERSION_PATCH 0)
set(MKEROS2_VERSION
    ${MKEROS2_VERSION_MAJOR}.${MKEROS2_VERSION_MINOR}.${MKEROS2_VERSION_PATCH})

add_definitions(-DMKEROS2_VERSION_MAJOR="${MKEROS2_VERSION_MAJOR}")
add_definitions(-DMKEROS2_VERSION_MINOR="${MKEROS2_VERSION_MINOR}")
add_definitions(-DMKEROS2_VERSION_PATCH="${MKEROS2_VERSION_PATCH}")
add_definitions(-DMKEROS2_VERSION="${MKEROS2_VERSION}")

set(BINNAME "mkeros2_node")
add_executable(${BINNAME} src/mkeros2_node.cpp)

if(MKECLI_TARGET)
  add_dependencies(${BINNAME} ${MKECLI_TARGET})
endif()

target_include_directories(${BINNAME} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_link_libraries(${BINNAME} ${MKECLI_LIBRARIES})
target_link_libraries(${BINNAME} ${CMAKE_THREAD_LIBS_INIT})

set(Boost_USE_STATIC_LIBS ON) 
# set(Boost_USE_MULTITHREADED OFF)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS system filesystem thread date_time REQUIRED) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
    target_link_libraries(${BINNAME} ${Boost_LIBRARIES})
endif()

set(dependencies "rclcpp" "std_srvs" "sensor_msgs")
ament_target_dependencies(${BINNAME} ${dependencies})

target_include_directories(${BINNAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${BINNAME}
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()