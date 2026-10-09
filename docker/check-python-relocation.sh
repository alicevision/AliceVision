#!/bin/bash
# Run with networking disabled (Docker RUN --network=none / docker run --network=none).
# Moves the runtime out of its original location, tests it, then restores it.
set -euo pipefail

runtime=$(realpath "${1:-/opt/python}")
expected_version=${2:-3.11.17}
test -x "${runtime}/bin/python3.11"
work=$(mktemp -d)
relocated="${work}/Meshroom_bundle/python"
mkdir -p "${work}/Meshroom_bundle" "${work}/home" "${work}/empty-path"

cleanup() {
    if [[ -d "${relocated}" ]]; then
        mv "${relocated}" "${runtime}"
    fi
    rm -rf "${work}"
}
trap cleanup EXIT

# No system interpreter on PATH, no activation, and no PYTHONHOME, PYTHONPATH,
# VIRTUAL_ENV, LD_LIBRARY_PATH or inherited pip/user configuration.
clean_python() {
    env -i HOME="${work}/home" PATH="${work}/empty-path" "$@"
}

check_interpreter() {
    clean_python "$1" -B - "$2" "$3" "${expected_version}" <<'PY'
import bz2
import ctypes
import ensurepip
import hashlib
import importlib.util
import json
import lzma
import sqlite3
import ssl
import sys
import sysconfig
import venv
import zlib
from pathlib import Path

prefix, base, version = sys.argv[1:]
assert sys.version.split()[0] == version, sys.version
assert Path(sys.prefix) == Path(prefix), (sys.prefix, prefix)
assert Path(sys.base_prefix) == Path(base), (sys.base_prefix, base)
assert Path(sysconfig.get_path("stdlib")).is_relative_to(base)
assert Path(ssl.__file__).is_relative_to(base), ssl.__file__
assert all(not p or Path(p).is_relative_to(prefix) or Path(p).is_relative_to(base)
           for p in sys.path), sys.path
assert importlib.util.find_spec("numpy") is None, "Build NumPy leaked into runtime"
assert ensurepip.version()
assert list((Path(ensurepip.__file__).parent / "_bundled").glob("pip-*.whl"))
assert Path(base, "lib/libpython3.11.so.1.0").is_file()
assert Path(base, "include/python3.11/Python.h").is_file()

metadata = json.loads(Path(base, "PYTHON.json").read_text())
assert metadata["target_triple"] == "x86_64-unknown-linux-gnu"
assert metadata["python_version"] == version

def check_licenses(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ("license_path", "license_paths"):
                for name in ([item] if isinstance(item, str) else item):
                    assert Path(base, name).is_file(), name
            else:
                check_licenses(item)
    elif isinstance(value, list):
        for item in value:
            check_licenses(item)

check_licenses(metadata)
for link in Path(base).rglob("*"):
    if link.is_symlink():
        assert link.resolve(strict=True).is_relative_to(base), link

assert bz2.decompress(bz2.compress(b"test")) == b"test"
assert lzma.decompress(lzma.compress(b"test")) == b"test"
assert zlib.decompress(zlib.compress(b"test")) == b"test"
assert sqlite3.connect(":memory:").execute("select 1").fetchone() == (1,)
assert ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
assert hashlib.sha256(b"test").hexdigest()
assert ctypes.CDLL(None)
if prefix != base:
    import pip
    assert Path(pip.__file__).is_relative_to(prefix), pip.__file__
    assert "include-system-site-packages = false" in Path(prefix, "pyvenv.cfg").read_text()
print(f"Python {version}: prefix={sys.prefix}, base_prefix={sys.base_prefix}")
PY
}

check_interpreter "${runtime}/bin/python3.11" "${runtime}" "${runtime}"
mv "${runtime}" "${relocated}"
test ! -e "${runtime}"
check_interpreter "${relocated}/bin/python3.11" "${relocated}" "${relocated}"
clean_python "${relocated}/bin/python3.11" -B -m pip --isolated --version

for mode in symlinks copies; do
    venv="${work}/venv-${mode}"
    # Default ensurepip bootstrapping uses ONLY the bundled wheels, offline.
    clean_python "${relocated}/bin/python3.11" -B -m venv "--${mode}" "${venv}"
    # Invoke the venv entry point itself, not its resolved base-interpreter path.
    check_interpreter "${venv}/bin/python" "${venv}" "${relocated}"
    clean_python "${venv}/bin/python" -B -m pip --isolated --version
    clean_python "${venv}/bin/python" -B -m pip --isolated check
done

echo "Standalone Python relocation and offline venv checks passed."