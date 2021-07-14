# This is MkE API C/C++ header files
# 2016-2021, Jan Heller, jan@magik-eye.com
 
cmake_minimum_required(VERSION 3.0)
project(mkeapi C CXX)

file(GLOB SOURCES "${CMAKE_CURRENT_LIST_DIR}/include/*")

install(FILES ${SOURCES} DESTINATION include/mke/api/)
