# Input: MKEAPI_ROOT - directory root where mkecli library has been installed
# MKEAPI_URL - path to the directory or archive containing the library sources
# MKEAPI_GIT - git repository containing the library sources Once done this will
# define MKEAPI_FOUND - System has Mke API headers 
# MKEAPI_INCLUDE_DIRS - MkE API headers directory

if(MKEAPI_URL OR MKEAPI_GIT)
  # Try compilation from source
  include(ExternalProject)
  set(MKEAPI_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/mkeapi)
  set(MKEAPI_TARGET "mkeapi")
  SET(MKEAPI_TARGET ${MKEAPI_TARGET} PARENT_SCOPE)

  if(MKEAPI_URL)
    ExternalProject_Add(
      ${MKEAPI_TARGET}
      URL ${MKEAPI_URL}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${MKEAPI_INSTALL_LOCATION}
                 -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  elseif(MKEAPI_GIT)
    ExternalProject_Add(
    ${MKEAPI_TARGET}
      GIT_REPOSITORY ${MKEAPI_GIT}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${MKEAPI_INSTALL_LOCATION}
                 -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  set(MKEAPI_INCLUDE_DIRS ${MKEAPI_INSTALL_LOCATION}/include/mke/api/)

else()

  # Find precompiled library
  find_path(
    MKEAPI_INCLUDE_DIR
    NAMES mkeapi.h
    HINTS ${MKEAPI_ROOT}
          ${CMAKE_CURRENT_SOURCE_DIR}/../../../mkeapi # git path
    PATH_SUFFIXES
      include
      include/mke/api/
    )

  include(FindPackageHandleStandardArgs)

  # handle the QUIETLY and REQUIRED arguments and set MKEAPI_FOUND to TRUE if
  # all listed variables are TRUE
  find_package_handle_standard_args(
    MKEAPI
    REQUIRED_VARS MKEAPI_INCLUDE_DIR
    FAIL_MESSAGE
      "Cannot find MkEAPI header files. Try setting MKEAPI_ROOT option to point to\n \
       the installation directory of the headers, or try to install from source by\n \
       setting MKEAPI_GIT or MKEAPI_URL options")

  mark_as_advanced(MKEAPI_INCLUDE_DIR)

  set(MKEAPI_INCLUDE_DIRS ${MKEAPI_INCLUDE_DIR})

endif()
