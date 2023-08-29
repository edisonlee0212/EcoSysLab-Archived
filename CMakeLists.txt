cmake_minimum_required(VERSION 3.17)
project(
        EcoSysLab
        VERSION 0.1
        DESCRIPTION "An interactive application developed for my research purposes based on EvoEngine"
)

option(BUILD_RAY_TRACER_FACILITY "Build Ray Tracer Facility" ON)
set(BUILD_RAY_TRACER_FACILITY ON)

# Set a default build type if none was specified
set(default_build_type "Release")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release")
endif ()

set(CMAKE_CXX_STANDARD 17)
if (WIN32)
    # Compiler settings for Windows platform
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
elseif (UNIX AND NOT APPLE)
    if (${CMAKE_BUILD_TYPE} STREQUAL Release)
        set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    elseif (${CMAKE_BUILD_TYPE} STREQUAL Debug)
        set(CMAKE_CXX_FLAGS_DEBUG "-g")
    endif ()
elseif (APPLE)
endif ()

# ------------------------------------------------------------------
# 3rd Party libraries
# ------------------------------------------------------------------
set(EVOENGINE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../EvoEngine)
if(BUILD_RAY_TRACER_FACILITY) 
    set(RAYTRACERFACILITY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../RayTracerFacility)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT TARGET evoengine)
    add_subdirectory(${EVOENGINE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/EvoEngine)
endif ()
if(BUILD_RAY_TRACER_FACILITY) 
    if (NOT TARGET raytracerfacility)
        add_subdirectory(${RAYTRACERFACILITY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/RayTracerFacility)
    endif ()
endif()

include_directories(${EVOENGINE_INCLUDES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


# ------------------------------------------------------------------
# ecosyslab
# ------------------------------------------------------------------
file(GLOB LOCAL_ECOSYSLAB_SOURCES "src/lib/*.cpp")
add_library(ecosyslab 
        STATIC
        ${LOCAL_ECOSYSLAB_SOURCES}
        )
set(LOCAL_ECOSYSLAB_PCH
        ${EVOENGINE_PCH}
        ${RAY_TRACER_FACILITY_PCH}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/EcoSysLab-pch.hpp
        )

target_precompile_headers(ecosyslab
        PRIVATE
        ${LOCAL_ECOSYSLAB_PCH}
        )
if (BUILD_RAY_TRACER_FACILITY)
    target_compile_definitions(ecosyslab
            PRIVATE
            NOMINMAX
            RAYTRACERFACILITY
            )
    set(LOCAL_ECOSYSLAB_INCLUDES
            ${RAY_TRACER_FACILITY_INCLUDES}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SorghumModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/ClimateModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SoilModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/TreeModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/PipeModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Utilities
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Structures
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Reconstruction
            )
    target_link_libraries(ecosyslab
            raytracerfacility
            )
else ()
    target_compile_definitions(ecosyslab
            PRIVATE
            NOMINMAX
            )
    set(LOCAL_ECOSYSLAB_INCLUDES
            ${EVOENGINE_INCLUDES}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SorghumModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/ClimateModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SoilModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/PipeModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/TreeModel
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Utilities
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Structures
            ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Reconstruction
            )
    target_link_libraries(ecosyslab
            evoengine
            )
endif ()
target_include_directories(ecosyslab
        PUBLIC
        ${LOCAL_ECOSYSLAB_INCLUDES}
        )


# ------------------------------------------------------------------
# ecosyslab - python binding
# ------------------------------------------------------------------
file(GLOB LOCAL_PYECOSYSLAB_SOURCES "src/python/*.cpp")
pybind11_add_module(pyecosyslab ${LOCAL_PYECOSYSLAB_SOURCES})

target_include_directories(pyecosyslab
            PRIVATE
            ${LOCAL_ECOSYSLAB_INCLUDES}
            ${CMAKE_CURRENT_SOURCE_DIR}/include/python
            )
target_precompile_headers(pyecosyslab
            PRIVATE
            ${LOCAL_ECOSYSLAB_PCH}
            )

if (BUILD_RAY_TRACER_FACILITY)
target_compile_definitions(pyecosyslab
            PRIVATE
            NOMINMAX
            RAYTRACERFACILITY
            )
target_link_libraries(pyecosyslab
            PRIVATE
            raytracerfacility
            ecosyslab
            )
else()
target_link_libraries(pyecosyslab
            PRIVATE
            evoengine
            ecosyslab
            )
endif()


# ------------------------------------------------------------------
# Example program
# ------------------------------------------------------------------
add_executable(ecosyslab-app
        "src/app/Application.cpp")
target_precompile_headers(ecosyslab-app
        PRIVATE
        ${LOCAL_ECOSYSLAB_PCH}
        )
if (BUILD_RAY_TRACER_FACILITY)
    target_link_libraries(ecosyslab-app
            raytracerfacility
            ecosyslab
            )
    target_compile_definitions(ecosyslab-app
            PRIVATE
            RAYTRACERFACILITY
            NOMINMAX
            )
else ()
    target_link_libraries(ecosyslab-app
            ecosyslab
            )
    target_compile_definitions(ecosyslab-app
            PRIVATE
            NOMINMAX
            )
endif ()

# ------------------------------------------------------------------
# Copy Internal resources
# ------------------------------------------------------------------
add_custom_command(TARGET ecosyslab POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Internals
        ${CMAKE_BINARY_DIR})

file(COPY src/app/imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})