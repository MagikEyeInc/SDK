# ##############################################################################
# pymkeviewer wheel

if(NOT DEFINED WHEELS_ROOT)
     set(STANDALONE TRUE)
endif()

if (STANDALONE)
     cmake_minimum_required(VERSION 3.1)
	set(WHEELS_ROOT ${CMAKE_CURRENT_BINARY_DIR})
     find_package(PythonInterp 3.6.0)
     find_package(PythonLibs 3.6.0 REQUIRED)     
endif()

set(OUTPUT_WHEEL_DIR "${WHEELS_ROOT}/pymkeviewer")
set(PKG_NAME "pymkeviewer")

set(VERSION_PATCH 5)
set(PYMKEVIEWER_VERSION
    ${MKERT_VERSION_MAJOR}.${MKERT_VERSION_MINOR}.${VERSION_PATCH})

file(MAKE_DIRECTORY ${OUTPUT_WHEEL_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PKG_NAME}
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/README.md
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PKG_NAME}/__init__.py
               ${CMAKE_CURRENT_BINARY_DIR}/${PKG_NAME}/__init__.py @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py
               ${CMAKE_CURRENT_BINARY_DIR}/setup.py @ONLY)

set(TARGET_NAME "${PKG_NAME}_wheel")

add_custom_target(
  ${TARGET_NAME} ALL
  # Build wheel
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/setup.py bdist_wheel
  # Copy wheel to python directory
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/dist/*
          ${CMAKE_CURRENT_BINARY_DIR} ${OUTPUT_WHEEL_DIR})

if (NOT STANDALONE)
     set(PYMKEVIEWER_VERSION "${PYMKEVIEWER_VERSION}" PARENT_SCOPE)
     set_property(GLOBAL PROPERTY PYMKEVIEWER_VERSION "${PYMKEVIEWER_VERSION}")

     get_property(MKERT_WHEELS_TARGETS GLOBAL PROPERTY MKERT_WHEELS_TARGETS)
     list(APPEND MKERT_WHEELS_TARGETS "${TARGET_NAME}")
     set_property(GLOBAL PROPERTY MKERT_WHEELS_TARGETS "${MKERT_WHEELS_TARGETS}")
endif()
