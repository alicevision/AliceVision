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
FROM ${DEPS_IMAGE}
LABEL maintainer="AliceVision Team alicevision-team@googlegroups.com"

ARG TARGET_ARCHITECTURE=core

ENV AV_DEV=/opt/AliceVision_git \
    AV_BUILD=/tmp/AliceVision_build \
    AV_INSTALL=/opt/AliceVision_install \
    AV_BUNDLE=/opt/AliceVision_bundle \
    PATH="${PATH}:${AV_BUNDLE}" \
    VERBOSE=1

COPY CMakeLists.txt *.md ${AV_DEV}/
COPY src ${AV_DEV}/src
COPY meshroom ${AV_DEV}/meshroom
COPY docker ${AV_DEV}/docker

WORKDIR "${AV_BUILD}"

# Configure, build, install and bundle in a single RUN so CPU_CORES (computed
# by check-cpu.sh) stays in scope for the build: a separate `RUN export ...`
# would be lost, leaving the build parallelism unbounded and exhausting memory.
RUN CPU_CORES=$(${AV_DEV}/docker/check-cpu.sh) && \
    echo "Build multithreading number of cores: ${CPU_CORES}" && \
    cmake -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS:BOOL=ON \
        -DTARGET_ARCHITECTURE=${TARGET_ARCHITECTURE} \
        -DALICEVISION_BUILD_DEPENDENCIES:BOOL=OFF \
        -DCMAKE_PREFIX_PATH:PATH="${AV_INSTALL}" \
        -DCMAKE_INSTALL_PREFIX:PATH="${AV_INSTALL}" \
        -DALICEVISION_BUNDLE_PREFIX="${AV_BUNDLE}" \
        -DALICEVISION_USE_ALEMBIC:BOOL=ON \
        -DMINIGLOG:BOOL=ON \
        -DALICEVISION_USE_CCTAG:BOOL=ON \
        -DALICEVISION_USE_OPENCV:BOOL=ON \
        -DALICEVISION_USE_POPSIFT:BOOL=ON \
        -DALICEVISION_USE_CUDA:BOOL=ON \
        -DALICEVISION_USE_ONNX_GPU:BOOL=OFF \
        -DALICEVISION_BUILD_DOC:BOOL=OFF \
        -DALICEVISION_BUILD_SWIG_BINDING:BOOL=ON \
        -DSWIG_DIR:PATH="${AV_INSTALL}/share/swig/4.3.0" -DSWIG_EXECUTABLE:PATH="${AV_INSTALL}/bin-deps/swig" \
        -DPython3_EXECUTABLE:PATH=/usr/bin/python \
        "${AV_DEV}" && \
    cmake --build . --target install --parallel "${CPU_CORES}" && \
    cmake --build . --target bundle && \
    rm -rf "${AV_BUILD}" "${AV_DEV}" && \
    echo "export ALICEVISION_SENSOR_DB=${AV_BUNDLE}/share/aliceVision/cameraSensors.db" >> /etc/profile.d/alicevision.sh && \
    echo "export ALICEVISION_ROOT=${AV_BUNDLE}" >> /etc/profile.d/alicevision.sh
