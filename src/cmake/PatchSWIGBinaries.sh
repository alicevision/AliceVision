#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "[ INFO ] Usage: $0 <directory>"
  exit 1
fi

ROOT_DIR="$1"

if [[ ! -d "$ROOT_DIR" ]]; then
  echo "[ ERROR ] '$ROOT_DIR' is not a directory"
  exit 1
fi

find "$ROOT_DIR" -type f -name "*.so" -print0 | while IFS= read -r -d '' file; do
  echo "[ INFO ] Patching SWIG binary file: $file"

  # Extract rpaths
  rpaths=$(otool -l "$file" 2>/dev/null | awk '
    /cmd LC_RPATH/ {in_rpath=1; next}
    in_rpath && /path / {
      print $2
      in_rpath=0
    }
  ')

  # Remove each existing rpath
  while IFS= read -r rp; do
    if [[ -n "$rp" ]]; then
      install_name_tool -delete_rpath "$rp" "$file" 2>/dev/null || true
    fi
  done <<< "$rpaths"

  # Add the new rpath
  install_name_tool -add_rpath "@loader_path/../../" "$file"

done

echo "[ INFO ] Done."
