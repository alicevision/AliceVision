include(BundleUtilities)

# Convert paths to unix style paths
function( to_cmake_paths _paths _out )
    foreach( src ${_paths} )
        file(TO_CMAKE_PATH ${src} result)
        list(APPEND _tmp "${result}")
    endforeach()
    set(${_out} ${_tmp} PARENT_SCOPE)
endfunction()

# Avoid issues with "\" on Windows
to_cmake_paths("${LPATHS}" UNIX_LPATHS)

# Perform the bundle fixup
fixup_bundle(${EXECUTABLE} "" "${UNIX_LPATHS}" )

