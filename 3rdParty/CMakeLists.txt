# ------------------------------------------------------------------
# uniengine
# ------------------------------------------------------------------
add_subdirectory(json)
add_subdirectory(csvpp)
add_subdirectory(xatlas)
add_subdirectory(base64)
set(NO_DX True)
set(TREEIO_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}/json
	${CMAKE_CURRENT_SOURCE_DIR}/xatlas
	${CMAKE_CURRENT_SOURCE_DIR}/csvpp
	${MAKE_CURRENT_SOURCE_DIR}/base64
		PARENT_SCOPE)

set(PINCHOT_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}/pinchot/includes
	PARENT_SCOPE)

set(PINCHOT_LIB
	${CMAKE_CURRENT_SOURCE_DIR}/pinchot/lib/windows/x64/pinchot.lib
	PARENT_SCOPE)

set(PINCHOT_DLL
	${CMAKE_CURRENT_SOURCE_DIR}/pinchot/lib/windows/x64/pinchot.dll
	PARENT_SCOPE)