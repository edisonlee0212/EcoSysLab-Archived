cmake_minimum_required(VERSION 3.17)

project(
		EcoSysLab
		VERSION 0.1
		DESCRIPTION "An interactive application developed for my research purposes based on EvoEngine"
)

# ------------------------------------------------------------------
# Build types
# ------------------------------------------------------------------
# Set a default build type if none was specified
set(default_build_type "RelWithDebInfo")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
			STRING "Choose the type of build." FORCE)
endif ()

if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]lWithDebInfo")
	set(DEBUG_STATUS NDEBUG)
elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
	set(DEBUG_STATUS _DEBUG)
endif ()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (WIN32)
	# Compiler settings for Windows platform
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
elseif (UNIX AND NOT APPLE)
	if (${CMAKE_BUILD_TYPE} STREQUAL Release)
		set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	elseif (${CMAKE_BUILD_TYPE} STREQUAL RelWithDebInfo)
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
	elseif (${CMAKE_BUILD_TYPE} STREQUAL Debug)
		set(CMAKE_CXX_FLAGS_DEBUG "-g")
	endif ()
elseif (APPLE)
endif ()

# ------------------------------------------------------------------
# 3rd Party libraries
# ------------------------------------------------------------------
set(EVOENGINE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../EvoEngine)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty)

# ------------------------------------------------------------------
# EvoEngine
# ------------------------------------------------------------------
add_subdirectory(${EVOENGINE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/EvoEngine)

# ------------------------------------------------------------------
# ecosyslab - includes and sources
# ------------------------------------------------------------------
file(GLOB ECOSYSLAB_SOURCES "src/lib/*.cpp")

set(ECOSYSLAB_INCLUDES
	${EVOENGINE_INCLUDES}
	${TREEIO_INCLUDES}
	${PINCHOT_INCLUDES}
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/JoeScan
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/MeshFormation
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Layers
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/TreeDescriptor
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SpatialCompetition
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/LogGrading
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/DataPipeline
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/BranchShape
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Physics
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/TreeIO
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Volumes
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SorghumModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/ClimateModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SoilModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/StrandModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/TreeModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/SpaceColonizationTreeModel
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Utilities
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Structures
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/Reconstruction
	)
set(ECOSYSLAB_PCH
	${EVOENGINE_PCHS}
	${CMAKE_CURRENT_SOURCE_DIR}/include/lib/EcoSysLab-pch.hpp
	)
message(${ECOSYSLAB_INCLUDES})
# ------------------------------------------------------------------
# ecosyslab
# ------------------------------------------------------------------

add_library(EcoSysLab 
	STATIC
	${ECOSYSLAB_SOURCES}
	)

target_include_directories(EcoSysLab
	PRIVATE
	${ECOSYSLAB_INCLUDES}
	)

target_precompile_headers(EcoSysLab
	PRIVATE
	${ECOSYSLAB_PCH}
	)

target_link_libraries(EcoSysLab
	PRIVATE
	EvoEngine_SDK
	${EvoEngine_Plugins}
	csvpp
	JSON
	xatlas
	Base64
	${PINCHOT_LIB}
	)

target_compile_definitions(EcoSysLab
	PRIVATE
	${EVOENGINE_DEFS}
	)



# ------------------------------------------------------------------
# ecosyslab - python binding
# ------------------------------------------------------------------
#file(GLOB PYECOSYSLAB_SOURCES "src/python/*.cpp")

#pybind11_add_module(pyecosyslab ${PYECOSYSLAB_SOURCES})

#target_include_directories(pyecosyslab
#	PRIVATE
#	${ECOSYSLAB_INCLUDES}
#	${CMAKE_CURRENT_SOURCE_DIR}/include/python
#	)

#target_precompile_headers(pyecosyslab
#	PRIVATE
#	${ECOSYSLAB_PCH}
#	)

#target_link_libraries(pyecosyslab
#	PRIVATE
#	ecosyslab
#	)

#target_compile_definitions(pyecosyslab
#	PRIVATE
#	NOMINMAX
#	)

# ------------------------------------------------------------------
# Example program
# ------------------------------------------------------------------
add_executable(EcoSysLab-app
		"src/app/EcoSysLabApplication.cpp")

target_include_directories(EcoSysLab-app
	PRIVATE
	${ECOSYSLAB_INCLUDES}
	)

target_precompile_headers(EcoSysLab-app
	PRIVATE
	${ECOSYSLAB_PCH}
	)

target_link_libraries(EcoSysLab-app
	PRIVATE
	EcoSysLab
	)

target_compile_definitions(EcoSysLab-app
	PRIVATE
	${EVOENGINE_DEFS}
	)

add_executable(SorghumFactory-app
		"src/app/SorghumFactoryApplication.cpp")

target_include_directories(SorghumFactory-app
	PRIVATE
	${ECOSYSLAB_INCLUDES}
	)

target_precompile_headers(SorghumFactory-app
	PRIVATE
	${ECOSYSLAB_PCH}
	)

target_link_libraries(SorghumFactory-app
	PRIVATE
	EcoSysLab
	)

target_compile_definitions(SorghumFactory-app
	PRIVATE
	${EVOENGINE_DEFS}
	)

add_executable(LogGrader-app
		"src/app/LogGraderApplication.cpp")

target_include_directories(LogGrader-app
	PRIVATE
	${ECOSYSLAB_INCLUDES}
	)

target_precompile_headers(LogGrader-app
	PRIVATE
	${ECOSYSLAB_PCH}
	)

target_link_libraries(LogGrader-app
	PRIVATE
	EcoSysLab
	)

target_compile_definitions(LogGrader-app
	PRIVATE
	${EVOENGINE_DEFS}
	)

add_executable(EcoSysLab-dataset
		"src/app/DatasetGeneration.cpp")

target_include_directories(EcoSysLab-dataset
	PRIVATE
	${ECOSYSLAB_INCLUDES}
	)

target_precompile_headers(EcoSysLab-dataset
	PRIVATE
	${ECOSYSLAB_PCH}
	)

target_link_libraries(EcoSysLab-dataset
	PRIVATE
	EcoSysLab
	)

target_compile_definitions(EcoSysLab-dataset
	PRIVATE
	${EVOENGINE_DEFS}
	)


# ------------------------------------------------------------------
# Copy Internal resources
# ------------------------------------------------------------------
add_custom_command(TARGET EcoSysLab POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_CURRENT_SOURCE_DIR}/src/Internals
	${CMAKE_BINARY_DIR})

add_custom_command(TARGET EcoSysLab POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${PINCHOT_DLL}
	${CMAKE_BINARY_DIR})

evoengine_copy_resources(EcoSysLab ${CMAKE_CURRENT_BINARY_DIR})

file(COPY src/app/imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})