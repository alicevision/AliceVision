# shellcheck shell=bash
# Container engine selection — sourced by the build-*.sh scripts.
#
# Picks the engine used to build/push the images. Override by exporting
# CONTAINER_ENGINE=docker|podman; otherwise docker is preferred and podman
# is used as a fallback. BuildKit (needed for the cache mounts in the
# Dockerfiles) is enabled automatically for docker; podman builds with
# buildah, which supports those mounts natively.

test -z "${CONTAINER_ENGINE:-}" && {
	if command -v docker >/dev/null 2>&1; then
		CONTAINER_ENGINE=docker
	elif command -v podman >/dev/null 2>&1; then
		CONTAINER_ENGINE=podman
	else
		echo "No container engine found: install docker or podman, or set CONTAINER_ENGINE." >&2
		exit 1
	fi
}

command -v "$CONTAINER_ENGINE" >/dev/null 2>&1 || {
	echo "CONTAINER_ENGINE='$CONTAINER_ENGINE' not found in PATH." >&2
	exit 1
}

if [ "$CONTAINER_ENGINE" = "docker" ]; then
	export DOCKER_BUILDKIT=1
fi

echo "CONTAINER_ENGINE: $CONTAINER_ENGINE"
