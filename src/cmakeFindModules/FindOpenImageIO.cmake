#find headers
FIND_PATH( OpenImageIO_INCLUDE_DIR OpenImageIO/imageio.h
  $ENV{OIIO_BASE}/include
  ${OIIO_BASE}}/include
  )

# find lib
FIND_LIBRARY( OpenImageIO_LIBRARY OpenImageIO
  $ENV{OIIO_ROOT}/lib
  ${OIIO_ROOT}/lib
  )

INCLUDE( FindPackageHandleStandardArgs )

FIND_PACKAGE_HANDLE_STANDARD_ARGS( "OpenImageIO" DEFAULT_MSG
  OpenImageIO_INCLUDE_DIR
  OpenImageIO_LIBRARY
  )