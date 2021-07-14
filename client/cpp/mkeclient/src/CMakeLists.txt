# This is libmkeclient: C++ library implementing MKE API client interface
# 2016--2021, Jan Heller, jan@magik-eye.com

# ##############################################################################
# libmkeclient
cmake_minimum_required(VERSION 3.1)

file(GLOB CLIENT_SOURCES "mke/cli/*")

add_library(
  ${LIBMKECLIENT} STATIC
  # SSDP
  mke/net/ssdp/discovery.cpp
  mke/net/ssdp/parser.cpp
  # Device
  mke/device/type.cpp
  mke/device/location.cpp
  # Client
  ${CLIENT_SOURCES}
  mke/cli/priv/rawcli.cpp
  # Util
  mke/util/crc32.cpp
)

# Execute mkeapi header installation before libmkeclient compilation
if(MKEAPI_TARGET)
  add_dependencies(${LIBMKECLIENT} ${MKEAPI_TARGET})
endif()

set_target_properties(${LIBMKECLIENT} PROPERTIES OUTPUT_NAME ${LIBMKECLIENT})
set_target_properties(${LIBMKECLIENT} PROPERTIES OUTPUT_NAME ${LIBMKECLIENT}
                                                 VERSION ${LIBMKECLIENT})

# set(MKECLI_INCLUDE_DIRS ".")
find_path(
  MKECLI_INCLUDE_DIRS
  NAMES mke/cli/client.h
  HINTS .
  PATH_SUFFIXES include)


# Boost includes
include_directories(${Boost_INCLUDE_DIRS})

# MkE API headers
target_include_directories(${LIBMKECLIENT} PRIVATE ${MKEAPI_INCLUDE_DIRS})
target_include_directories(${LIBMKECLIENT} PRIVATE ${MKECLI_INCLUDE_DIRS})

# ##############################################################################
# Demo

if(MKECLI_DEMO)
  add_subdirectory(examples)
endif()

# ##############################################################################
# mkecli_tester

if(MKECLI_TESTER)
  set(BINNAME "mkecli_tester")
  file(GLOB TESTER_SOURCES "tester/*.cpp")
  file(GLOB TESTER_TESTS "tester/tests_*/*.cpp") 
  add_executable(${BINNAME} ${TESTER_SOURCES} ${TESTER_TESTS})

  target_link_libraries(${BINNAME} ${LIBMKECLIENT})
  target_include_directories(${BINNAME} PRIVATE ${MKEAPI_INCLUDE_DIRS})
  target_include_directories(${BINNAME} PRIVATE ${MKECLI_INCLUDE_DIRS})
endif()

# ##############################################################################
# Installation

install(TARGETS ${LIBMKECLIENT} DESTINATION lib)

# Install MKEAPI headers as well

install(
FILES
  ${MKEAPI_INCLUDE_DIRS}/mkeapi.h
DESTINATION
  include/mke/api/
)

install(
FILES
  ${MKEAPI_INCLUDE_DIRS}/mkeapi_reserved.h
DESTINATION
  include/mke/api/
OPTIONAL  
)

install(
FILES 
  mke/net/ssdp/discovery.h
DESTINATION 
  include/mke/net/ssdp/
)

install(
FILES 
  mke/error.h
DESTINATION 
  include/mke/
)

install(
FILES 
  mke/memory/ptrpool.h
  mke/memory/boostptrpool.h
  mke/memory/mempool.h
DESTINATION
  include/mke/memory/
)

install(
FILES 
  mke/device/type.h
  mke/device/location.h
DESTINATION 
  include/mke/device/
)

install(
FILES
  mke/cli/error.h
  mke/cli/client.h
  mke/cli/bus.h
  mke/cli/factory.h
DESTINATION
  include/mke/cli/
)

install(
FILES
  mke/cli/reserved.h
DESTINATION
  include/mke/cli/
OPTIONAL
)

install(
FILES
  mke/cli/priv/rawcli.h
DESTINATION
  include/mke/cli/priv/
)
