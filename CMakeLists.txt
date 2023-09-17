cmake_minimum_required(VERSION 3.17)
project(
		EcoSysLab
		VERSION 0.1
		DESCRIPTION "An interactive application developed for my research purposes based on EvoEngine"
)

option(BUILD_WITH_RAYTRACER "Build with Ray Tracer" OFF)
option(BUILD_WITH_PHYSICS "Build with Physics" OFF)

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
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
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
if (NOT TARGET evoengine)
	if (NOT TARGET evoengine)
		add_subdirectory(${EVOENGINE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/EvoEngine)
	endif()
endif ()
if(BUILD_WITH_RAYTRACER) 
	set(EVOENGINE_RAYTRACER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../EvoEngine-RayTracer)
	if (NOT TARGET evoengine-raytracer)
		add_subdirectory(${EVOENGINE_RAYTRACER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/EvoEngine-RayTracer)
	endif ()
endif()
if(BUILD_WITH_PHYSICS) 
	set(EVOENGINE_PHYSICS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../EvoEngine-Physics)
	if (NOT TARGET evoengine-physics)
		add_subdirectory(${EVOENGINE_PHYSICS_DIR} ${CMAKE_CURRENT_BINARY_DIR}/EvoEngine-Physics)
	endif ()
endif()

# ------------------------------------------------------------------
# ecosyslab - includes and sources
# ------------------------------------------------------------------
file(GLOB LOCAL_ECOSYSLAB_SOURCES "src/lib/*.cpp")
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
set(LOCAL_ECOSYSLAB_PCH
	${EVOENGINE_PCH}
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/EcoSysLab-pch.hpp
	)

# ------------------------------------------------------------------
# ecosyslab
# ------------------------------------------------------------------

add_library(ecosyslab 
	STATIC
	${LOCAL_ECOSYSLAB_SOURCES}
	)

target_precompile_headers(ecosyslab
	PRIVATE
	${LOCAL_ECOSYSLAB_PCH}
	)

target_include_directories(ecosyslab
	PRIVATE
	${LOCAL_ECOSYSLAB_INCLUDES}
	)

target_link_libraries(ecosyslab
	PRIVATE
	evoengine
	)

target_compile_definitions(ecosyslab
	PRIVATE
	NOMINMAX
	)

if (BUILD_WITH_RAYTRACER)
	target_include_directories(ecosyslab
		PRIVATE
		${EVOENGINE_RAYTRACER_INCLUDES}
	)

	target_link_libraries(ecosyslab
		PRIVATE
		evoengine-raytracer
		)

	target_compile_definitions(ecosyslab
		PRIVATE
		BUILD_WITH_RAYTRACER
		)
endif ()

if (BUILD_WITH_PHYSICS)
	target_include_directories(ecosyslab
		PRIVATE
		${EVOENGINE_PHYSICS_INCLUDES}
	)

	target_link_libraries(ecosyslab
		PRIVATE
		evoengine-physics
		)

	target_compile_definitions(ecosyslab
		PRIVATE
		BUILD_WITH_PHYSICS
		)
endif ()

# ------------------------------------------------------------------
# ecosyslab - python binding
# ------------------------------------------------------------------
file(GLOB LOCAL_PYECOSYSLAB_SOURCES "src/python/*.cpp")

pybind11_add_module(pyecosyslab ${LOCAL_PYECOSYSLAB_SOURCES})

target_precompile_headers(pyecosyslab
	PRIVATE
	${LOCAL_ECOSYSLAB_PCH}
	)

target_include_directories(pyecosyslab
	PRIVATE
	${LOCAL_ECOSYSLAB_INCLUDES}
	${CMAKE_CURRENT_SOURCE_DIR}/include/python
	)

target_link_libraries(pyecosyslab
	PRIVATE
	ecosyslab
	)

target_compile_definitions(pyecosyslab
	PRIVATE
	NOMINMAX
	)

if (BUILD_WITH_RAYTRACER)
	target_include_directories(pyecosyslab
		PRIVATE
		${EVOENGINE_RAYTRACER_INCLUDES}
	)
	target_link_libraries(pyecosyslab
		PRIVATE
		evoengine-raytracer
		)

	target_compile_definitions(pyecosyslab
		PRIVATE
		BUILD_WITH_RAYTRACER
		)
endif()

if (BUILD_WITH_PHYSICS)
	target_include_directories(pyecosyslab
		PRIVATE
		${EVOENGINE_PHYSICS_INCLUDES}
	)
	target_link_libraries(pyecosyslab
		PRIVATE
		evoengine-physics
		)

	target_compile_definitions(pyecosyslab
		PRIVATE
		BUILD_WITH_PHYSICS
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
target_include_directories(ecosyslab-app
	PRIVATE
	${LOCAL_ECOSYSLAB_INCLUDES}
	)

target_link_libraries(ecosyslab-app
	PRIVATE
	ecosyslab
	)

target_compile_definitions(ecosyslab-app
	PRIVATE
	NOMINMAX
	)

if (BUILD_WITH_RAYTRACER)
	target_include_directories(ecosyslab-app
		PRIVATE
		${EVOENGINE_RAYTRACER_INCLUDES}
	)
	target_link_libraries(ecosyslab-app
		PRIVATE
		evoengine-raytracer
		)
	target_compile_definitions(ecosyslab-app
		PRIVATE
		BUILD_WITH_RAYTRACER
		)
endif ()

if (BUILD_WITH_PHYSICS)
	target_include_directories(ecosyslab-app
		PRIVATE
		${EVOENGINE_PHYSICS_INCLUDES}
	)
	target_link_libraries(ecosyslab-app
		PRIVATE
		evoengine-physics
		)
	target_compile_definitions(ecosyslab-app
		PRIVATE
		BUILD_WITH_PHYSICS
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