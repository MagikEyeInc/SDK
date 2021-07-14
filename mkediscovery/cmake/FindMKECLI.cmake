# * Try to find libmkeclient libraries and includes
#
# Input: MKECLI_ROOT - directory root where mkecli library has been installed
#        MKECLI_URL - path to the directory or archive containing the library
#                     sources
#        MKECLI_GIT - git repository containing the library sources.
#        MKEAPI_URL - path to the directory containing MKE API headers project
# define MKECLI_FOUND - System has libmkeclient
#        MKECLI_INCLUDE_DIRS - mkeclient library include directory
#        MKECLI_LIBRARIES - The libraries needed to use libmkeclient

# Try to find libmkeclient sources in the repo
if(NOT (MKECLI_URL OR MKECLI_GIT OR MKECLI_ROOT))
  find_path(
    MKECLI_DIR
    NAMES client.h
    HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../../ # git path
          ${CMAKE_CURRENT_SOURCE_DIR}/../../ # git sdk path
          ${CMAKE_CURRENT_SOURCE_DIR}/../
    PATH_SUFFIXES 
      cpp/mkeclient/src/mke/cli/ 
  )

  if (MKECLI_DIR)
    set (MKECLI_URL "${MKECLI_DIR}/../../../")
  endif()
endif()

if(MKECLI_URL OR MKECLI_GIT)
  # Try compilation from source
  include(ExternalProject)
  set(MKECLI_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/mkecli)
  set(MKECLI_TARGET "mkecli")

  # Try to find MkE API headers in the repo
  if (NOT (MKEAPI_URL OR MKEAPI_GIT))
    find_path(
      MKEAPI_INCLUDE_DIR
      NAMES mkeapi.h
      HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../../../ # git path
            ${CMAKE_CURRENT_SOURCE_DIR}/../../../ # git sdk path
      PATH_SUFFIXES 
        mkeapi/include
    )

    if (MKEAPI_INCLUDE_DIR)
      set(MKEAPI_URL "${MKEAPI_INCLUDE_DIR}/../")
    endif()
  endif()

  if (NOT MKEAPI_URL)
    set(MKEAPI_URL "")
  endif()
  
  if (NOT MKEAPI_GIT)
    set(MKEAPI_GIT "")
  endif()
    
  if(MKECLI_URL)
    ExternalProject_Add(
      ${MKECLI_TARGET}
      URL ${MKECLI_URL}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${MKECLI_INSTALL_LOCATION}
                 -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                 -DMKEAPI_URL=${MKEAPI_URL}
                 -DMKEAPI_GIT=${MKEAPI_GIT}
      )
  elseif(MKECLI_GIT)
    ExternalProject_Add(
      ${MKECLI_TARGET}
      GIT_REPOSITORY ${MKECLI_GIT}
      SOURCE_SUBDIR "client/cpp/mkeclient/"
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${MKECLI_INSTALL_LOCATION}
                 -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                 -DMKEAPI_URL=${MKEAPI_URL}
                 -DMKEAPI_GIT=${MKEAPI_GIT}
      )
  endif()

  set(MKECLI_LIBRARIES ${MKECLI_INSTALL_LOCATION}/lib/libmkeclient.a)
  set(MKECLI_INCLUDE_DIRS 
    ${MKECLI_INSTALL_LOCATION}/include/
    ${MKECLI_INSTALL_LOCATION}/include/mke/api
  )

else()
  # Find precompiled library
  find_path(
    MKECLI_INCLUDE_DIR
    NAMES mke/cli/client.h
    HINTS ${MKECLI_ROOT}
    PATH_SUFFIXES include)

  find_library(
    MKECLI_LIBRARY
    NAMES libmkeclient.a
    HINTS ${MKECLI_ROOT}
    PATH_SUFFIXES lib)

  include(FindPackageHandleStandardArgs)

  # handle the QUIETLY and REQUIRED arguments and set MKECLI_FOUND to TRUE if
  # all listed variables are TRUE
  find_package_handle_standard_args(
    MKECLI
    REQUIRED_VARS MKECLI_LIBRARY MKECLI_INCLUDE_DIR
    FAIL_MESSAGE
      "Cannot find the mkeclient library. Try setting MKECLI_ROOT option to point\n  \
       the installation directory of the library, or try to compile from source by\n \
       setting MKECLI_GIT or MKECLI_URL options")

  mark_as_advanced(MKECLI_INCLUDE_DIR MKECLI_LIBRARY)

  set(MKECLI_INCLUDE_DIRS ${MKECLI_INCLUDE_DIR})
  set(MKECLI_LIBRARIES ${MKECLI_LIBRARY})

endif()
