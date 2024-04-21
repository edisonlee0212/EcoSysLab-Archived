cmake_minimum_required (VERSION 3.1)
project(configure_and_connect)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
include_directories(${ROOT_DIR})

if (WIN32)
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  add_library(joescanapi SHARED IMPORTED)

  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # 64 bit
  set_property(
    TARGET pinchot
    PROPERTY IMPORTED_LOCATION ${ROOT_DIR}/lib/windows10/x64/pinchot.dll)
  set_property(
    TARGET pinchot
    PROPERTY IMPORTED_IMPLIB ${ROOT_DIR}/lib/windows10/x64/pinchot.lib)

  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    # 32 bit
  set_property(
    TARGET pinchot
    PROPERTY IMPORTED_LOCATION ${ROOT_DIR}/lib/windows10/x86/pinchot.dll)
  set_property(
    TARGET pinchot
    PROPERTY IMPORTED_IMPLIB ${ROOT_DIR}/lib/windows10/x86/pinchot.lib)

  endif()
endif (WIN32)

add_executable(configure_and_connect configure_and_connect.cpp)
target_link_libraries(configure_and_connect pinchot)

