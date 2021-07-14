cmake_minimum_required(VERSION 3.0.2)
project(mkeros)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################

catkin_package(
  INCLUDE_DIRS CATKIN_DEPENDS roscpp std_msgs
)

# ##############################################################################
# Build ##
# ##############################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
find_package(MKECLI REQUIRED)

set(MKEROS1_VERSION_MAJOR 1)
set(MKEROS1_VERSION_MINOR 0)
set(MKEROS1_VERSION_PATCH 0)
set(MKEROS1_VERSION
    ${MKEROS1_VERSION_MAJOR}.${MKEROS1_VERSION_MINOR}.${MKEROS1_VERSION_PATCH})

add_definitions(-DMKEROS1_VERSION_MAJOR="${MKEROS1_VERSION_MAJOR}")
add_definitions(-DMKEROS1_VERSION_MINOR="${MKEROS1_VERSION_MINOR}")
add_definitions(-DMKEROS1_VERSION_PATCH="${MKEROS1_VERSION_PATCH}")
add_definitions(-DMKEROS1_VERSION="${MKEROS1_VERSION}")

set(BINNAME "mkeros1_node")
add_executable(${BINNAME} src/mkeros1_node.cpp)

install(FILES
  launch/mkeros1_cpp.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  config/mkeros1_config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if(MKECLI_TARGET)
  add_dependencies(${BINNAME} ${MKECLI_TARGET})
endif()

target_include_directories(${BINNAME} PRIVATE ${MKECLI_INCLUDE_DIRS})
target_link_libraries(${BINNAME} ${MKECLI_LIBRARIES})

target_include_directories(${BINNAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${BINNAME} ${catkin_LIBRARIES})

install(TARGETS ${BINNAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
