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

set(DEP_EIGEN_VERSION  "5.0.1")
set(DEP_EIGEN_URL      "https://gitlab.com/libeigen/eigen/-/archive/${DEP_EIGEN_VERSION}/eigen-${DEP_EIGEN_VERSION}.tar.bz2")
set(DEP_EIGEN_HASH     "MD5=457323cbf3688f52b3623c0aa85044b7")

set(DEP_BOOST_VERSION  "1.92.0")
set(DEP_BOOST_URL      "https://github.com/boostorg/boost/releases/download/boost-${DEP_BOOST_VERSION}/boost-${DEP_BOOST_VERSION}-cmake.tar.xz")
set(DEP_BOOST_HASH     "SHA256=9bed76128d4e46755dbe818487788c6fceb6f72b378f4daa49b7e1e600d9088d")

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

set(DEP_CUDA_VERSION   "12.8.0")
set(DEP_CUDA_DRIVER    "570.86.10")
set(DEP_CUDA_URL       "https://developer.download.nvidia.com/compute/cuda/${DEP_CUDA_VERSION}/local_installers/cuda_${DEP_CUDA_VERSION}_${DEP_CUDA_DRIVER}_linux.run")
# No hash — NVIDIA does not publish stable checksums for installer scripts

# ── Image codecs ──────────────────────────────────────────────────────────────

set(DEP_TIFF_GIT_TAG   "v4.7.2")
set(DEP_TIFF_GIT_REPO  "https://gitlab.com/libtiff/libtiff.git")

set(DEP_PNG_VERSION    "1.6.58")
set(DEP_PNG_URL        "https://download.sourceforge.net/libpng/libpng-${DEP_PNG_VERSION}.tar.gz")
set(DEP_PNG_HASH       "MD5=40aaee5111ff68814d57351e68f15f29")

set(DEP_JPEG_VERSION   "3.2.0")
set(DEP_JPEG_URL       "https://github.com/libjpeg-turbo/libjpeg-turbo/archive/${DEP_JPEG_VERSION}.tar.gz")
set(DEP_JPEG_HASH      "MD5=47d465f8ba76031a6717afc70c91eaf3")

set(DEP_LIBRAW_CMAKE_GIT_REPO "https://github.com/LibRaw/LibRaw-cmake")
set(DEP_LIBRAW_CMAKE_GIT_TAG  "eb98e4325aef2ce85d2eb031c2ff18640ca616d3")
set(DEP_LIBRAW_GIT_REPO       "https://github.com/LibRaw/LibRaw")
set(DEP_LIBRAW_GIT_TAG        "0.22.2")

set(DEP_OPENEXR_VERSION "3.4.15")
set(DEP_OPENEXR_URL     "https://github.com/AcademySoftwareFoundation/openexr/archive/v${DEP_OPENEXR_VERSION}.tar.gz")
set(DEP_OPENEXR_HASH    "MD5=f54150623ed23c2783f1aa5f680c2b7f")

# ── Video ─────────────────────────────────────────────────────────────────────

set(DEP_VPX_GIT_REPO   "https://chromium.googlesource.com/webm/libvpx.git")
set(DEP_VPX_GIT_TAG    "v1.15.2")

set(DEP_FFMPEG_VERSION "8.1")
set(DEP_FFMPEG_URL     "http://ffmpeg.org/releases/ffmpeg-${DEP_FFMPEG_VERSION}.tar.bz2")
set(DEP_FFMPEG_HASH    "MD5=bd1de4317d0fdcdb4c058b9139971aae")

# ── Color / Image processing ──────────────────────────────────────────────────

set(DEP_ONNXRUNTIME_VERSION "1.30.0")
# Per-platform hashes — resolved in color_image.cmake based on host OS/arch
set(DEP_ONNXRUNTIME_LINUX_X64_HASH    "SHA256=a5ed5a3cac51fbb2e90da632ae43d19212faaa20e76484e62bcb7c23ddb3b3fd")
set(DEP_ONNXRUNTIME_LINUX_AARCH64_HASH "SHA256=e16a27a8ed330bbc698df7330b0cf56e722f354e3bcc92118682c74ef3c3e3da")
set(DEP_ONNXRUNTIME_OSX_ARM64_HASH    "SHA256=6ebb5062a934537c352937821f9fe9718e7de1a2db1122a93dd363ffd53a7012")

set(DEP_OPENCOLORIO_GIT_REPO "https://github.com/AcademySoftwareFoundation/OpenColorIO.git")
set(DEP_OPENCOLORIO_GIT_TAG  "v2.5.2")

set(DEP_OPENIMAGEIO_VERSION "3.1.17.0")
set(DEP_OPENIMAGEIO_URL     "https://github.com/AcademySoftwareFoundation/OpenImageIO/archive/refs/tags/v${DEP_OPENIMAGEIO_VERSION}.tar.gz")
set(DEP_OPENIMAGEIO_HASH    "MD5=6129ccf733f6cc6c9d14550d3765d9af")

set(DEP_OPENCV_VERSION "4.14.0")
set(DEP_OPENCV_URL         "https://github.com/opencv/opencv/archive/refs/tags/${DEP_OPENCV_VERSION}.tar.gz")
set(DEP_OPENCV_HASH        "MD5=5b382fc2e99fb7c1eb2b929b0c447ed4")
set(DEP_OPENCV_CONTRIB_URL "https://github.com/opencv/opencv_contrib/archive/refs/tags/${DEP_OPENCV_VERSION}.tar.gz")
set(DEP_OPENCV_CONTRIB_HASH "MD5=773c8306ce1fba536e6bf16a406e769f")

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

set(DEP_SUITESPARSE_VERSION "7.14.0")
set(DEP_SUITESPARSE_URL     "https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v${DEP_SUITESPARSE_VERSION}.tar.gz")
set(DEP_SUITESPARSE_HASH    "MD5=4f1de135ee1afb0d5383f59316c26489")

set(DEP_CERES_GIT_REPO "https://github.com/ceres-solver/ceres-solver")
set(DEP_CERES_GIT_TAG  "fe351d5")

set(DEP_LZ4_GIT_REPO   "https://github.com/lz4/lz4")
set(DEP_LZ4_GIT_TAG    "v1.9.4")

set(DEP_FLANN_GIT_REPO "https://github.com/alicevision/flann")
set(DEP_FLANN_GIT_TAG  "46e72429ef60ce9c413fa926ac7729f8dee96395")

set(DEP_NANOFLANN_GIT_REPO "https://github.com/jlblancoc/nanoflann")
set(DEP_NANOFLANN_GIT_TAG  "92911c0bc382e4b287330219bc720ca2b30b2857")

set(DEP_COINUTILS_GIT_REPO "https://github.com/alicevision/CoinUtils")
set(DEP_COINUTILS_GIT_TAG  "b29532e31471d26dddee99095da3340e80e8c60c")

set(DEP_OSI_GIT_REPO "https://github.com/alicevision/Osi")
set(DEP_OSI_GIT_TAG  "52bafbabf8d29bcfd57818f0dd50ee226e01db7f")

set(DEP_CLP_GIT_REPO "https://github.com/alicevision/Clp")
set(DEP_CLP_GIT_TAG  "4da587acebc65343faafea8a134c9f251efab5b9")

set(DEP_LEMON_GIT_REPO "https://github.com/alicevision/lemon.git")
set(DEP_LEMON_GIT_TAG  "5493d5317605f56076e94b04caa6199d52957d74")

# ── 3D / Geometry ─────────────────────────────────────────────────────────────

set(DEP_GEOGRAM_VERSION "1.9.6")
set(DEP_GEOGRAM_URL     "https://github.com/BrunoLevy/geogram/releases/download/v${DEP_GEOGRAM_VERSION}/geogram_${DEP_GEOGRAM_VERSION}.tar.gz")
set(DEP_GEOGRAM_HASH    "MD5=ca4f42cbda64d8fb386708150dac7057")

set(DEP_ASSIMP_VERSION "5.4.3")
set(DEP_ASSIMP_URL     "https://github.com/assimp/assimp/archive/refs/tags/v${DEP_ASSIMP_VERSION}.tar.gz")
set(DEP_ASSIMP_HASH    "MD5=fd64a9a57a3d81940ba7fc4a3a946502")

set(DEP_ALEMBIC_VERSION "1.8.12")
set(DEP_ALEMBIC_URL     "https://github.com/alembic/alembic/archive/${DEP_ALEMBIC_VERSION}.tar.gz")
set(DEP_ALEMBIC_HASH    "MD5=1f0f4e9fcd92f104dcdf825c6a08773f")

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
set(DEP_USD_GIT_TAG  "v26.08")

set(DEP_OPENSUBDIV_GIT_REPO "https://github.com/PixarAnimationStudios/OpenSubdiv.git")
set(DEP_OPENSUBDIV_GIT_TAG  "v3_7_0")

# ── Feature detectors ─────────────────────────────────────────────────────────

set(DEP_POPSIFT_GIT_REPO "https://github.com/alicevision/popsift")
set(DEP_POPSIFT_GIT_TAG  "36d704d39b4cc065839d84f3706b3fa88eff2518")

set(DEP_CCTAG_GIT_REPO "https://github.com/alicevision/CCTag")
set(DEP_CCTAG_GIT_TAG  "71021443af3f7946e8e9025a37634b57e7aad77d")

set(DEP_APRILTAG_GIT_REPO "https://github.com/AprilRobotics/apriltag")
set(DEP_APRILTAG_GIT_TAG  "v3.2.0")
