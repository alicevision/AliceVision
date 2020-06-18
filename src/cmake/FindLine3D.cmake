# Find Line3D
# ------------
#
# Find Line3D include dirs and libraries
#
# This module defines the following variables:
#
#   Geogram_FOUND        - True if geogram has been found.
#   Line3D::line3D       - Imported target for the main Line3D library.
#
# This module reads hints about the Line3D location from the following
# environment variables:
#
#   LINE3D_INSTALL_PREFIX - Directory where Line3D is installed.
#

set (LINE3D_SEARCH_PATHS
  ${LINE3D_INSTALL_PREFIX}                
  "$ENV{LINE3D_INSTALL_PREFIX}"
)

find_path (LINE3D_INCLUDE_DIR
                line3D/line3D.h
                PATHS ${LINE3D_SEARCH_PATHS}
                PATH_SUFFIXES include
)

find_library (LINE3D_LIBRARY
                NAMES line3Dpp
                PATHS ${LINE3D_SEARCH_PATHS}
                PATH_SUFFIXES lib
)


include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  Line3D DEFAULT_MSG LINE3D_LIBRARY LINE3D_INCLUDE_DIR
)

# Create an imported target for Line3D 
If (Line3D_FOUND)

    if (NOT TARGET Line3D::line3D)
        add_library (Line3D::line3D UNKNOWN IMPORTED)

        # Interface include directory
        set_target_Properties(Line3D::line3D PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${LINE3D_INCLUDE_DIR}"
        )

        # Link to library file
        Set_Target_Properties(Line3D::line3D PROPERTIES
          IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
          IMPORTED_LOCATION ${LINE3D_LIBRARY}
        )
    endif ()
endif ()

# Hide variables from the default CMake-Gui options
mark_as_advanced (LINE3D_LIBRARY LINE3D_INCLUDE_DIR)
