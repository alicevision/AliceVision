# imagedb: tiny inputs covering every code path of `aliceVision_cameraInit`

110 cases run `aliceVision_cameraInit` on extra-small inputs (16x12 JPEGs, 32x24 DNGs, hand-written LCP/DCP profiles,
`.sfm` files) and compare the result with a recorded reference. Together they execute every reachable line of the
camera initialization code; the lines that are not executed are listed below with the reason.

Two uses:
- **characterization test** of today's behaviour, run by ctest (`test_aliceVision_cameraInit_imagedb`);
- **regression harness** for a refactoring of camera initialization: every output must match the reference, or
  differ in a way that is explained (then re-record with `--update`).

## Files

| Path | What |
|---|---|
| `run_cases.py` | Runner (standard library only). Runs each case, checks exit status, log substrings and output. |
| `cases.json` | The cases: arguments, expected status (`ok`, `fail`, `abort`), expected log substrings, branch ids covered. |
| `expected/<case>.json` | Normalized reference output of each `ok` case writing `.sfm`. |
| `generate.py` | Writes `data/` and `cases.json` (dev only; output is committed). JPEGs need Pillow. |
| `tifflib.py` | Minimal TIFF writer used for the DNG and DCP files. |
| `data/img`, `data/folders` | Real image files: EXIF variants, folder layouts (rigs, pose groups, frame ids, nested folders). |
| `data/raw`, `data/dcp` | Generated DNGs (LibRaw-readable, 32x24: LibRaw rejects images under 22 pixels) and DCP profiles. |
| `data/lcp` | LCP profiles: element style, attribute style, nested descriptions, fisheye, mixed, broken. |
| `data/sfm` | `.sfm` inputs carrying metadata a JPEG can't hold (Sony, ARRI, DJI keys, malformed numbers, ids). |
| `data/sensors.db` | Small sensor database (exact, inexact, negative widths, comment and short lines). |

Why `.sfm` inputs: when a view of the input `.sfm` already has metadata, cameraInit does not read the image metadata
(`viewIO.cpp`, `bootstrapView`); the image file must still exist (size, raw detection), so those views all point at
`data/img/base.jpg`. Paths are written as `{DATA}/...` and rewritten by the runner, so the dataset is relocatable.

## Running

```bash
# in the build environment of AliceVision
python3 imagedb/run_cases.py --bin <build>/Linux-x86_64/aliceVision_cameraInit
python3 imagedb/run_cases.py --bin ... --only lcp_ --keep /tmp/imagedb_out   # subset, keep outputs and logs
python3 imagedb/run_cases.py --bin ... --update                             # re-record expected/
python3 imagedb/run_cases.py --bin ... --skip output_format_abc             # build without Alembic

# regenerate the data (after editing generate.py), then re-record
rez env pillow-12.1.1 -- python3 imagedb/generate.py
```

The runner disables core dumps for the cases that abort (each would write a core of several hundred MB), runs the
binary with `LC_ALL=C` and without `ALICEVISION_LOG_LEVEL`, `ALICEVISION_ROOT` and `ALICEVISION_SENSOR_DB` (each case
sets its own), and treats exit codes 126/127 (binary not run) as an error even for the cases expected to fail. Each
failing case also checks its error message, so that it can't pass by failing for another reason.

ctest registers the test on UNIX only (the references hold POSIX paths, signal exits and glibc's `nan` formatting),
and skips `output_format_abc` / `output_format_usda` when AliceVision is built without Alembic / USD.

### What is compared

The `.sfm` output is normalized before comparison:
- view ids, pose ids, intrinsic ids, rig ids and image group ids are hashes of absolute paths or file times. They are
  replaced by the relative path of the view (or of the first view using the pose/intrinsic/rig/image group), so the
  comparison checks **which views share a pose, an intrinsic, a rig or an image group**, not the hash values; a
  reference to an image group that is not written is reported as missing. Views of an input `.sfm` that share one image
  file are told apart by their (stable) view id from the `.sfm`. The rig id that cameraInit appends to the intrinsic
  serial number (`_rig_<rigId>_<subPoseId>`) is replaced the same way;
- absolute data paths become `{DATA}`;
- metadata written by OpenImageIO itself (`oiio:*`, `jpeg:*`, file `DateTime`, resolution keys) is dropped;
- numbers are compared with a relative tolerance of 1e-6.

The results do not depend on the number of threads (checked with `OMP_NUM_THREADS` from 1 to 12) nor on the build type
(Release and Debug give the same outputs). For that, `lcp_multi` and `malformed_numbers` give different focal lengths
to views of one camera that differ only by distortion or initialization mode: the intrinsic id ignores both, so such
views would share an id and the intrinsic kept would depend on thread order.

Nor do they depend on where the dataset is or on the file times (checked from another folder, after `touch`).
For that, every pose group without an ambient view (`folders/posegroups`) has distinct timestamps: cameraInit sorts
the views by (timestamp, view id), and the view id is a hash of the absolute path and of the file time, so a tie
would pick the middle view at random.

## Branch map

Ids used in `cases.json` (`covers`). Line numbers are those of develop @ `21af5486f`.

| Id | Code | Branch |
|---|---|---|
| A1-A13 | `main_cameraInit.cpp` 252-345, `parseDatabase.cpp` | command line checks: no/unknown option, model strings, missing or both inputs, missing folder/sfm, empty output, output folder creation, focal + FOV, focal ratio, sensor DB from option / `ALICEVISION_SENSOR_DB` (priority) / `ALICEVISION_ROOT` / missing / bad width |
| L2-L9 | `jsonIO.cpp`, `main_cameraInit.cpp` 62-97, 372-405 | sfm version, malformed sfm, dangling intrinsic id, single file input, unsupported files, recursive listing, nothing found, no views |
| B1-B7 | `viewIO.cpp` 27-116 | fully set-up view skipped, unreadable image, metadata from sfm vs file, view id from file name regex (and its errors) or metadata, rig view without pose id, pose id different from view id |
| UID | `uid.cpp` | image unique id, serials, folder fallback, original date, `imageCounter`, file time, EXIF dimensions |
| R1-R7, F1, P1-P3, X1 | `main_cameraInit.cpp` 433-490, 760-851, 927 | rig detection (`rig/<n>/`), invalid rig folders, sub-pose gaps, unequal counts, frame ids from file stems, `ps_`/`hdr_` pose groups (ambient view, median timestamp), duplicate frame ids |
| D1-D9, C1 | `main_cameraInit.cpp` 506-562, 795-806, `dcp.cpp` | raw detection, DCP interpretations, database missing/empty/no match, cache, 1 or 2 illuminants, forward and calibration matrices, invalid profiles, optional DCP tags, every illuminant value |
| I1-I2 | `main_cameraInit.cpp` 565-580 | intrinsic already in the input sfm: kept, or rebuilt (focal 0 / not pinhole) |
| S1-S13 | `ImageInfo.cpp` `getSensorSize` | database hit / inexact hit / miss, 35mm keys (classic and Sony), width from diagonal, focal from 35mm, Sony effective size (and its trailing-space height key), unknown sensor, no metadata, 36 mm default, portrait ratio |
| FL1-FL5 | `ImageInfo.cpp` `getMetadataFocalLength` | generic keys, Sony meters, sentinel values, DJI lists, ARRI nominal focal |
| MD | `ImageInfo.cpp/.hpp` | alternative metadata key spellings, `ApertureValue` when `FNumber` is not numeric |
| LC1-LC14 | `main_cameraInit.cpp` 611-650, `lcp.cpp` | LCP folder walk, single profile, search by make/model/lens/lens id, caches, camera/lens/lens id/raw profile matching, broken XML, no geometry, interpolation in focal and aperture, vignetting and chromatic metadata |
| G1-G16 | `viewIO.cpp` `getViewIntrinsic` | default focal/FOV, resized/rotated EXIF dimensions, missing focal, init mode, pixel aspect ratio, fisheye heuristic, LCP model, user camera/distortion models, GoPro presets, LCP coefficients, `Make == "Custom"` |
| N1-N9 | `main_cameraInit.cpp` 666-756 | serial number from body/lens serial, folder/image/global fallback, make/model suffix, rig suffix, `_FocalLengthMM_` suffix, intrinsic id hash, initialized count |
| W1-W4 | `main_cameraInit.cpp` 856-950, `sfmDataIO.cpp` | warning lists, not enough initialized views, image set vs sequence, output formats (sfm, json, ply, baf, abc, usda) and invalid extension |

## Coverage

Measured with a separate Debug build compiled with `--coverage`, after running all the cases, with gcovr 7.2
(`--exclude-throw-branches --exclude-unreachable-branches`). Scope: the functions executed by cameraInit in
`main_cameraInit.cpp`, `viewIO.cpp`, `ImageInfo.cpp/.hpp`, `uid.cpp`, `parseDatabase.cpp`, `Datasheet.cpp`, `lcp.cpp`,
`dcp.cpp` and `sfmDataIO.cpp`.

**1928 of 2119 lines are executed. All the 191 others are listed here:**

| Lines | Why not executed |
|---|---|
| `dcp.cpp` `sget2`/`sget4` big-endian branches, `Tag::toInt`/`toDouble` (types other than SHORT, LONG, SRATIONAL, FLOAT), `Tag::toString` for non-ASCII tags (101 lines) | Generic TIFF tag decoding, not camera initialization: DCP profiles use the types already covered. Out of scope. |
| `ImageInfo.cpp` 216-236, 290-295, 310-315, 346-363, 384-385 (31) | Logging under `if (verbose)`: cameraInit calls `getSensorSize(..., false)`. |
| `lcp.cpp` 653, 682-688, 712-718, 753-772, 787 (22) | Focus distance refinement: cameraInit always searches with a focus distance of 0. |
| `lcp.cpp` 806-811 | Unreachable: `iLow != iHigh` with equal focal lengths cannot happen (both indices use the same tie rule). |
| `lcp.cpp` 860-861 | Unreachable: `search` only returns entries with valid vignetting parameters. |
| `lcp.cpp` 522 | Allocation failure of the XML parser. |
| `lcp.cpp` 552-553 | Executed by `lcp_broken`, which aborts (exception inside an OpenMP region): gcov writes no data on abort. The runner checks the message instead. |
| `viewIO.cpp` 34-57, 103-106 | Same: executed by the `viewid_filename_*` and `sfm_rig_without_pose` cases, which abort; checked through their log. |
| `viewIO.cpp` 145-147 | Unreachable: `getSensorSize` always returns a sensor width >= 0. |
| `uid.cpp` 85 | Dead code: `DateTime` is already one of the "original date" keys tested just before. |
| `dcp.cpp` 913 | `fopen` failing on a profile file that was just listed (unreadable file; git can't store that). |
| `dcp.cpp` 2375 | `DCPDatabase::load` without `force`: cameraInit always forces. |
| `main_cameraInit.cpp` 114 | Invalid enum value. |
| `main_cameraInit.cpp` 310-311 | `create_directory` throws instead of returning false. |
| `main_cameraInit.cpp` 779-780 | Unreachable: a negative sub-pose id is caught by the check above (`>= nbSubPose`). |
| `main_cameraInit.cpp` 227, 233; `lcp.cpp` 1159, 1169; `viewIO.cpp` 339 | gcov artifacts (continuation lines of a string literal, closing braces). |

Branches: in the decision code (`main_cameraInit.cpp` 338/356, `viewIO.cpp` 175/198, `ImageInfo.cpp` 194/206,
`ImageInfo.hpp` 199/199, `uid.cpp` 60/61), the untaken branches are: `if (verbose)`, `assert`, the abort-only paths
above, impossible conditions (`lensParam == nullptr`, intrinsic not `IntrinsicScaleOffset`, init mode `NONE`), and
compiler-generated edges of temporary strings in conditions.

### Reproducing the measure

```bash
# configure a second tree of the same sources with coverage flags (Debug), build only the executable
cmake -G Ninja -S <repo> -B <cov> -C <initial cache of the normal build> -DCMAKE_BUILD_TYPE=Debug \
      "-DCMAKE_CXX_FLAGS=--coverage" -DCMAKE_EXE_LINKER_FLAGS=--coverage -DCMAKE_SHARED_LINKER_FLAGS=--coverage \
      -DALICEVISION_BUILD_TESTS=OFF -DALICEVISION_BUILD_SWIG_BINDING=OFF
ninja -C <cov> aliceVision_cameraInit
python3 imagedb/run_cases.py --bin <cov>/Linux-x86_64/aliceVision_cameraInit
cd /tmp && gcovr -r <repo> --object-directory <cov> --filter '<repo>/src/software/pipeline/main_cameraInit.cpp' \
      --filter '<repo>/src/aliceVision/sfmDataIO/viewIO.cpp' ... --exclude-throw-branches \
      --exclude-unreachable-branches --txt
```

Run gcovr outside the repository: it writes temporary `.gcov` files in the current folder.
