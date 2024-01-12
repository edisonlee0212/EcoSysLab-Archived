# ------------------------------------------------------------------
# uniengine
# ------------------------------------------------------------------
add_subdirectory(json)
add_subdirectory(csvpp)
add_subdirectory(base64)
set(NO_DX True)
add_subdirectory(OpenSubdiv)
set(TREEIO_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/json
        ${CMAKE_CURRENT_SOURCE_DIR}/csvpp
        ${CMAKE_CURRENT_SOURCE_DIR}/base64
        ${CMAKE_CURRENT_SOURCE_DIR}/OpenSubdiv
        PARENT_SCOPE)