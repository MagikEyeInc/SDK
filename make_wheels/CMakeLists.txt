# MKE SDK Python wheels build script 2021, Jan Heller, jan@magik-eye.com

cmake_minimum_required(VERSION 3.1)
include(version.cmake)

set(PYTHON_VERSION
    ""
    CACHE STRING "Required python version for bindings")

if(PYTHON_VERSION)
  set(Python_ADDITIONAL_VERSIONS ${PYTHON_VERSION})
  find_package(PythonInterp ${PYTHON_VERSION} EXACT)
  find_package(PythonLibs ${PYTHON_VERSION} EXACT REQUIRED)
else()
  find_package(PythonInterp 3.6.0)
  find_package(PythonLibs 3.6.0 REQUIRED)
endif()

set(WHEELS_ROOT "${CMAKE_CURRENT_BINARY_DIR}/wheels")

# ##############################################################################

file(GLOB SRC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../client/python/py*)

foreach(DIR ${SRC_DIRS})
    if(IS_DIRECTORY ${DIR})
        get_filename_component(DIR_NAME ${DIR} NAME)
        add_subdirectory(${DIR} ${DIR_NAME})
    endif()
endforeach()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pymkeviewer/pymkeviewer
                 pymkeviewer)
