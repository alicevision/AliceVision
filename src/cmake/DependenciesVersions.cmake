# =============================================================================
# versions.cmake
#
# Single source of truth for all external dependency versions, download URLs
# and integrity hashes.
#
# Naming conventions:
#   DEP_<LIB>_VERSION          — version string (used to build URLs and find paths)
#   DEP_<LIB>_URL              — full download URL
#   DEP_<LIB>_HASH             — checksum in the form <ALGO>=<value>
#   DEP_<LIB>_GIT_REPO         — git remote URL  (for git-based deps)
#   DEP_<LIB>_GIT_TAG          — git tag / commit (for git-based deps)
#
# To upgrade a dependency: edit only this file.
# =============================================================================

# ── Core / Compression ────────────────────────────────────────────────────────

set(DEP_ZLIB_VERSION   "1.3.2")
set(DEP_ZLIB_URL       "https://www.zlib.net/zlib-${DEP_ZLIB_VERSION}.tar.gz")
set(DEP_ZLIB_HASH      "SHA256=bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16")

set(DEP_TBB_VERSION    "2022.1.0-rc1")
set(DEP_TBB_URL        "https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v${DEP_TBB_VERSION}.tar.gz")
set(DEP_TBB_HASH       "MD5=e37f0538269b454c1bf2b5356c2bb617")

set(DEP_EIGEN_VERSION  "3.4.0")
set(DEP_EIGEN_URL      "https://gitlab.com/libeigen/eigen/-/archive/${DEP_EIGEN_VERSION}/eigen-${DEP_EIGEN_VERSION}.tar.bz2")
set(DEP_EIGEN_HASH     "MD5=132dde48fe2b563211675626d29f1707")

set(DEP_BOOST_VERSION  "1.86.0")
set(DEP_BOOST_URL      "https://github.com/boostorg/boost/releases/download/boost-${DEP_BOOST_VERSION}/boost-${DEP_BOOST_VERSION}-cmake.tar.xz")
set(DEP_BOOST_HASH     "SHA256=2c5ec5edcdff47ff55e27ed9560b0a0b94b07bd07ed9928b476150e16b0efc57")

set(DEP_EXPAT_GIT_REPO "https://github.com/libexpat/libexpat.git")
set(DEP_EXPAT_GIT_TAG  "R_2_7_4")

set(DEP_PYBIND11_GIT_REPO "https://github.com/pybind/pybind11.git")
set(DEP_PYBIND11_GIT_TAG  "v2.13.6")

set(DEP_SWIG_GIT_REPO  "https://github.com/swig/swig")
set(DEP_SWIG_GIT_TAG   "v4.3.0")
# cmake find_package path uses the version string
set(DEP_SWIG_VERSION   "4.3.0")

set(DEP_OPENMP_VERSION "22.1.3")
set(DEP_OPENMP_URL     "https://github.com/llvm/llvm-project/releases/download/llvmorg-${DEP_OPENMP_VERSION}/llvm-project-${DEP_OPENMP_VERSION}.src.tar.xz")
set(DEP_OPENMP_HASH    "MD5=1b8f0fdca6f49e323702ed7d4da0feae")

# ── CUDA ──────────────────────────────────────────────────────────────────────

set(DEP_CUDA_VERSION   "12.1.1")
set(DEP_CUDA_DRIVER    "530.30.02")
set(DEP_CUDA_URL       "https://developer.download.nvidia.com/compute/cuda/${DEP_CUDA_VERSION}/local_installers/cuda_${DEP_CUDA_VERSION}_${DEP_CUDA_DRIVER}_linux.run")
# No hash — NVIDIA does not publish stable checksums for installer scripts

# ── Image codecs ──────────────────────────────────────────────────────────────

set(DEP_TIFF_GIT_TAG   "v4.5.1")
set(DEP_TIFF_GIT_REPO  "https://gitlab.com/libtiff/libtiff.git")

set(DEP_PNG_VERSION    "1.6.50")
set(DEP_PNG_URL        "https://download.sourceforge.net/libpng/libpng-${DEP_PNG_VERSION}.tar.gz")
set(DEP_PNG_HASH       "MD5=eef2d3da281ae83ac8a8f5fd9fa9d325")

set(DEP_JPEG_VERSION   "2.1.5.1")
set(DEP_JPEG_URL       "https://github.com/libjpeg-turbo/libjpeg-turbo/archive/${DEP_JPEG_VERSION}.tar.gz")
set(DEP_JPEG_HASH      "MD5=33f72421d83ba487ff7b5c81e8765185")

set(DEP_LIBRAW_CMAKE_GIT_REPO "https://github.com/LibRaw/LibRaw-cmake")
set(DEP_LIBRAW_CMAKE_GIT_TAG  "6e26c9e73677dc04f9eb236a97c6a4dc225ba7e8")
set(DEP_LIBRAW_GIT_REPO       "https://github.com/LibRaw/LibRaw")
set(DEP_LIBRAW_GIT_TAG        "0.21.1")

set(DEP_OPENEXR_VERSION "3.1.6")
set(DEP_OPENEXR_URL     "https://github.com/AcademySoftwareFoundation/openexr/archive/v${DEP_OPENEXR_VERSION}.tar.gz")
set(DEP_OPENEXR_HASH    "MD5=da5daf4d7954c034921e7201bf815938")

# ── Video ─────────────────────────────────────────────────────────────────────

set(DEP_VPX_GIT_REPO   "https://chromium.googlesource.com/webm/libvpx.git")
set(DEP_VPX_GIT_TAG    "v1.15.2")

set(DEP_FFMPEG_VERSION "6.1.4")
set(DEP_FFMPEG_URL     "http://ffmpeg.org/releases/ffmpeg-${DEP_FFMPEG_VERSION}.tar.bz2")
set(DEP_FFMPEG_HASH    "MD5=588be1baa8ca8cf6861bcae88020f0f9")

# ── Color / Image processing ──────────────────────────────────────────────────

set(DEP_ONNXRUNTIME_VERSION "1.17.0")
# Per-platform hashes — resolved in color_image.cmake based on host OS/arch
set(DEP_ONNXRUNTIME_LINUX_X64_HASH    "SHA256=efc344d54d1969446ff5d3e55b54e205c6579c06333ecf1d34a04215eefae7c6")
set(DEP_ONNXRUNTIME_LINUX_AARCH64_HASH "SHA256=ee5069252f549ef94759b6b60bdf10b2dc2cd71d064a7045dd66a052f956a68b")
set(DEP_ONNXRUNTIME_OSX_ARM64_HASH    "SHA256=f72a2bcca40e2650756c6b96c69ef031236aaab1b98673e744da4eef0c4bddbd")
set(DEP_ONNXRUNTIME_OSX_X86_64_HASH   "SHA256=b87b2febef24e5645e13859d176e76473124325a0b1526baf7f68b4aa1eb1b49")

set(DEP_OPENCOLORIO_GIT_REPO "https://github.com/AcademySoftwareFoundation/OpenColorIO.git")
set(DEP_OPENCOLORIO_GIT_TAG  "v2.4.2")

set(DEP_OPENIMAGEIO_VERSION "3.0.17.0")
set(DEP_OPENIMAGEIO_URL     "https://github.com/AcademySoftwareFoundation/OpenImageIO/archive/refs/tags/v${DEP_OPENIMAGEIO_VERSION}.tar.gz")
set(DEP_OPENIMAGEIO_HASH    "MD5=6d6b611b22117a23ac0b9dd0c0ec0ff6")

set(DEP_OPENCV_VERSION "4.12.0")
set(DEP_OPENCV_URL         "https://github.com/opencv/opencv/archive/refs/tags/${DEP_OPENCV_VERSION}.tar.gz")
set(DEP_OPENCV_HASH        "MD5=eb6f8ff4f4cd16ef1b97bc21edc74de9")
set(DEP_OPENCV_CONTRIB_URL "https://github.com/opencv/opencv_contrib/archive/refs/tags/${DEP_OPENCV_VERSION}.tar.gz")
set(DEP_OPENCV_CONTRIB_HASH "MD5=55603c033cc5f3d5e307b699ad72e25a")

# ── Math / Solvers ────────────────────────────────────────────────────────────

set(DEP_LAPACK_VERSION "3.11.0")
set(DEP_LAPACK_URL     "https://github.com/Reference-LAPACK/lapack/archive/v${DEP_LAPACK_VERSION}.tar.gz")
set(DEP_LAPACK_HASH    "MD5=595b064fd448b161cd711fe346f498a7")

set(DEP_GMP_VERSION    "6.2.1")
set(DEP_GMP_URL        "https://gmplib.org/download/gmp/gmp-${DEP_GMP_VERSION}.tar.xz")
set(DEP_GMP_HASH       "MD5=0b82665c4a92fd2ade7440c13fcaa42b")

set(DEP_MPFR_VERSION   "4.2.0")
set(DEP_MPFR_URL       "https://ftp.gnu.org/gnu/mpfr/mpfr-${DEP_MPFR_VERSION}.tar.gz")
set(DEP_MPFR_HASH      "MD5=279b527503118a22bd0022e0d64807cb")

set(DEP_SUITESPARSE_VERSION "7.3.0")
set(DEP_SUITESPARSE_URL     "https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v${DEP_SUITESPARSE_VERSION}.tar.gz")
set(DEP_SUITESPARSE_HASH    "MD5=6ff86003a85d73eb383d82db04af7373")

set(DEP_CERES_GIT_REPO "https://github.com/ceres-solver/ceres-solver")
set(DEP_CERES_GIT_TAG  "2.2.0")

set(DEP_LZ4_GIT_REPO   "https://github.com/lz4/lz4")
set(DEP_LZ4_GIT_TAG    "v1.9.4")

set(DEP_FLANN_GIT_REPO "https://github.com/alicevision/flann")
set(DEP_FLANN_GIT_TAG  "46e72429ef60ce9c413fa926ac7729f8dee96395")

set(DEP_NANOFLANN_GIT_REPO "https://github.com/jlblancoc/nanoflann")
set(DEP_NANOFLANN_GIT_TAG  "419c26c498d12231817ada6488e2fd2442dbc68d")

set(DEP_COINUTILS_GIT_REPO "https://github.com/alicevision/CoinUtils")
set(DEP_COINUTILS_GIT_TAG  "b29532e31471d26dddee99095da3340e80e8c60c")

set(DEP_OSI_GIT_REPO "https://github.com/alicevision/Osi")
set(DEP_OSI_GIT_TAG  "52bafbabf8d29bcfd57818f0dd50ee226e01db7f")

set(DEP_CLP_GIT_REPO "https://github.com/alicevision/Clp")
set(DEP_CLP_GIT_TAG  "4da587acebc65343faafea8a134c9f251efab5b9")

set(DEP_LEMON_GIT_REPO "https://github.com/alicevision/lemon.git")
set(DEP_LEMON_GIT_TAG  "8885b9a8b7a20cdf5588964fe30da89093ec53cd")

# ── 3D / Geometry ─────────────────────────────────────────────────────────────

set(DEP_GEOGRAM_VERSION "1.9.6")
set(DEP_GEOGRAM_URL     "https://github.com/BrunoLevy/geogram/releases/download/v${DEP_GEOGRAM_VERSION}/geogram_${DEP_GEOGRAM_VERSION}.tar.gz")
set(DEP_GEOGRAM_HASH    "MD5=ca4f42cbda64d8fb386708150dac7057")

set(DEP_ASSIMP_VERSION "5.2.5")
set(DEP_ASSIMP_URL     "https://github.com/assimp/assimp/archive/refs/tags/v${DEP_ASSIMP_VERSION}.tar.gz")
set(DEP_ASSIMP_HASH    "MD5=0b5a5a2714f1126b9931cdb95f512c91")

set(DEP_ALEMBIC_VERSION "1.8.5")
set(DEP_ALEMBIC_URL     "https://github.com/alembic/alembic/archive/${DEP_ALEMBIC_VERSION}.tar.gz")
set(DEP_ALEMBIC_HASH    "MD5=fcd5b5492a005057e11b601b60ac9a49")

set(DEP_XERCESC_VERSION "3.3.0")
set(DEP_XERCESC_URL     "https://downloads.apache.org/xerces/c/3/sources/xerces-c-${DEP_XERCESC_VERSION}.tar.xz")
set(DEP_XERCESC_HASH    "MD5=7efbd9d785551c71d44ab6782e30c3c4")

set(DEP_E57FORMAT_GIT_REPO "https://github.com/asmaloney/libE57Format.git")
set(DEP_E57FORMAT_GIT_TAG  "v3.2.0")

set(DEP_OPENMESH_VERSION "10.0.0")
set(DEP_OPENMESH_URL     "https://www.graphics.rwth-aachen.de/media/openmesh_static/Releases/10.0/OpenMesh-${DEP_OPENMESH_VERSION}.tar.bz2")
set(DEP_OPENMESH_HASH    "MD5=4d166aecbc09df58b38de9759c92a437")

set(DEP_PCL_VERSION "1.15.1")
set(DEP_PCL_URL     "https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DEP_PCL_VERSION}.tar.gz")
set(DEP_PCL_HASH    "MD5=e29ad2147fbe2109233e2b3a0254dbab")

set(DEP_USD_GIT_REPO "https://github.com/PixarAnimationStudios/USD.git")
set(DEP_USD_GIT_TAG  "v25.08")

set(DEP_OPENSUBDIV_GIT_REPO "https://github.com/PixarAnimationStudios/OpenSubdiv.git")
set(DEP_OPENSUBDIV_GIT_TAG  "v3_7_0")

# ── Feature detectors ─────────────────────────────────────────────────────────

set(DEP_POPSIFT_GIT_REPO "https://github.com/alicevision/popsift")
set(DEP_POPSIFT_GIT_TAG  "77d1d28624d2849798fc107619507b50cd8cfe10")

set(DEP_CCTAG_GIT_REPO "https://github.com/alicevision/CCTag")
set(DEP_CCTAG_GIT_TAG  "ff01840224a7f0948230f74bda723c9e0a216e5b")

set(DEP_APRILTAG_GIT_REPO "https://github.com/AprilRobotics/apriltag")
set(DEP_APRILTAG_GIT_TAG  "v3.2.0")
