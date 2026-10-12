# syntax=docker/dockerfile:1.4

# =============================================================================
# Dockerfile
#
# Builds AliceVision on top of a prebuilt alicevision-deps image (Rocky or
# Ubuntu). The dependencies image is passed as DEPS_IMAGE; this single file
# replaces the former Dockerfile_rocky / Dockerfile_ubuntu, which only differed
# by their base image.
#
# Built by docker/build-image.sh (which sets DEPS_IMAGE to the deps tag).
# =============================================================================

ARG DEPS_IMAGE

# Only what the build needs, so unrelated changes in the context (.git, docs...)
# do not invalidate the build cache. Bind-mounted below, never in the image.
FROM scratch AS sources
COPY CMakeLists.txt *.md /
COPY src /src
COPY meshroom /meshroom
COPY docker /docker

FROM ${DEPS_IMAGE}
LABEL maintainer="AliceVision Team alicevision-team@googlegroups.com"

ARG TARGET_ARCHITECTURE=core
# Build-time only (an ARG is not kept in the image): full compile command lines.
ARG VERBOSE=1
# One build tree per OS/CUDA flavour; set by docker/build-image.sh.
ARG AV_BUILD_CACHE_ID=default

ENV AV_DEV=/opt/AliceVision_git         \
    AV_BUILD=/tmp/AliceVision_build     \
    AV_INSTALL=/opt/AliceVision_install \
    AV_BUNDLE=/opt/AliceVision_bundle

# ENV rather than /etc/profile.d, which only login shells read: plain
# `docker run <image> aliceVision_...` must see them too.
# The bundled libs have no RPATH (fixup_bundle strips it on Linux), so they only
# find each other through LD_LIBRARY_PATH, as when Meshroom runs the bundle.
ENV PATH="${AV_BUNDLE}/bin:${PATH}"                                          \
    ALICEVISION_ROOT="${AV_BUNDLE}"                                          \
    ALICEVISION_SENSOR_DB="${AV_BUNDLE}/share/aliceVision/cameraSensors.db" \
    LD_LIBRARY_PATH="${AV_BUNDLE}/lib:${LD_LIBRARY_PATH}"

WORKDIR "${AV_BUILD}"

# Configure, build, install and bundle in a single RUN so CPU_CORES (computed
# by check-cpu.sh) stays in scope for the build: a separate `RUN export ...`
# would be lost, leaving the build parallelism unbounded and exhausting memory.
# The sources are bind-mounted rather than COPY'd so they never end up in an
# image layer (rw: build-time writes are discarded). Path hardcoded = ${AV_DEV}.
# The build tree lives in a BuildKit cache mount, kept across builds: a source
# change only recompiles what it affects. The install prefix and the bundle are
# regular layers, recreated by every build on top of the deps image.
RUN --mount=type=bind,from=sources,target=/opt/AliceVision_git,rw \
    --mount=type=cache,id=alicevision-build-${AV_BUILD_CACHE_ID},target=${AV_BUILD},sharing=locked \
    CPU_CORES=$(${AV_DEV}/docker/check-cpu.sh) &&                 \
    echo "Build multithreading number of cores: ${CPU_CORES}" &&  \
    cmake -DCMAKE_BUILD_TYPE=Release                              \
        -DCMAKE_GENERATOR=Ninja                                   \
        -DBUILD_SHARED_LIBS:BOOL=ON                               \
        -DTARGET_ARCHITECTURE=${TARGET_ARCHITECTURE}              \
        -DALICEVISION_BUILD_DEPENDENCIES:BOOL=OFF                 \
        -DCMAKE_PREFIX_PATH:PATH="${AV_INSTALL}"                  \
        -DCMAKE_INSTALL_PREFIX:PATH="${AV_INSTALL}"               \
        -DALICEVISION_BUNDLE_PREFIX="${AV_BUNDLE}"                \
        -DALICEVISION_USE_ALEMBIC:BOOL=ON                         \
        -DMINIGLOG:BOOL=ON                                        \
        -DALICEVISION_USE_CCTAG:BOOL=ON                           \
        -DALICEVISION_USE_OPENCV:BOOL=ON                          \
        -DALICEVISION_USE_POPSIFT:BOOL=ON                         \
        -DALICEVISION_USE_CUDA:BOOL=ON                            \
        -DALICEVISION_USE_ONNX_GPU:BOOL=OFF                       \
        -DALICEVISION_BUILD_DOC:BOOL=OFF                          \
        -DALICEVISION_BUILD_SWIG_BINDING:BOOL=ON                  \
        -DPython3_EXECUTABLE:FILEPATH="${PYTHON_BUILD_VENV:-/usr}/bin/python" \
        -DSWIG_EXECUTABLE:PATH="${AV_INSTALL}/bin-deps/swig"      \
        "${AV_DEV}" &&                                            \
    cmake --build . --target install --parallel "${CPU_CORES}" && \
    cmake --build . --target bundle &&                            \
    { ! test -x /opt/python/bin/python3.11 ||                     \
      /opt/python/bin/python3.11 -c 'import importlib.util; assert importlib.util.find_spec("numpy") is None'; }
