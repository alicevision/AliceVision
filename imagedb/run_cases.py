#!/usr/bin/env python3
"""Run every imagedb case through aliceVision_cameraInit and compare with the recorded results.

Standard library only (run by ctest).

    python3 imagedb/run_cases.py --bin <path/to/aliceVision_cameraInit> [--update] [--only NAME] [--keep DIR]

For each case of cases.json:
  - "{DATA}" / "{OUT}" in the arguments are replaced by the data folder and a temporary output folder; input .sfm files
    are copied with "{DATA}" replaced, so the dataset is relocatable;
  - the exit status is checked ("ok", "fail", "abort", "fail-or-abort"); exit codes 126/127 (binary not run) never
    count as "fail";
  - the expected log substrings must appear in the output (every "fail"/"abort" case has some, so that a failure for
    another reason is not taken for the expected one);
  - for "ok" cases, the output .sfm is normalized and compared with expected/<case>.json (--update rewrites them).

Normalization keeps what cameraInit decides and drops what depends on the machine:
  - view ids, pose ids, intrinsic ids, rig ids and image group ids are hashes of absolute paths or file times:
    they are replaced by the relative path of the view (or of the first view using that pose, intrinsic, rig or
    image group), so the comparison checks the groupings, not the hash values; the same goes for the rig id that
    cameraInit appends to the intrinsic serial number ("_rig_<rigId>_<subPoseId>");
  - absolute data paths become "{DATA}";
  - metadata written by OpenImageIO itself ("oiio:*", "jpeg:*", file DateTime) is dropped;
  - numbers are compared with a relative tolerance.
"""
import argparse
import json
import logging
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
logger = logging.getLogger("imagedb")

DROPPED_METADATA_PREFIXES = ("oiio:", "jpeg:", "Exif:ExifVersion", "Exif:FlashPixVersion", "Exif:ColorSpace",
                             "Exif:YCbCrPositioning", "ResolutionUnit", "XResolution", "YResolution")
DROPPED_METADATA_KEYS = {"DateTime"}
REL_TOL = 1e-6


def substitute(value, data, out):
    return value.replace("{DATA}", str(data)).replace("{OUT}", str(out))


def prepare_args(case, data, out, workdir):
    args = []
    for a in case["args"]:
        a = substitute(a, data, out)
        p = Path(a)
        if a.endswith(".sfm") and p.is_file() and p.parent == data / "sfm":
            copied = workdir / p.name
            copied.write_text(p.read_text().replace("{DATA}", str(data)))
            a = str(copied)
        args.append(a)
    if case.get("defaultOutput", True) and not any(a in ("-o", "--output") for a in args):
        args += ["--output", str(out / "cameraInit.sfm")]
    if case["sensorDb"]:
        args += ["--sensorDatabase", str(data / case["sensorDb"])]
    return args


def output_path(args):
    for i, a in enumerate(args[:-1]):
        if a in ("-o", "--output"):
            return Path(args[i + 1])
    return None


def _no_core_dump():
    """Child process setup: the "abort" cases would otherwise write a core file (hundreds of MB) each."""
    import resource
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def classify(returncode):
    if returncode == 0:
        return "ok"
    if returncode in (126, 127):
        return "not run"  # shell codes: not executable / not found (e.g. missing shared library)
    if returncode < 0:
        # std::terminate ends in abort(); any other signal (segmentation fault...) is a crash, never expected
        return "abort" if returncode == -signal.SIGABRT else f"signal {-returncode}"
    return "fail"


def expected_matches(expect, actual):
    if expect == "fail-or-abort":
        return actual in ("fail", "abort")
    return expect == actual


# ---------------------------------------------------------------------------------------------------------------------
# Normalization and comparison
# ---------------------------------------------------------------------------------------------------------------------

def normalize(sfm, data):
    data_str = str(data)

    def rel(path):
        return path.replace(data_str, "{DATA}")

    def clean(value):
        if isinstance(value, str):
            return rel(value)
        if isinstance(value, list):
            return [clean(v) for v in value]
        if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items()}
        return value

    views = sfm.get("views", [])
    # Views are keyed by relative path. Several views of an input .sfm may share one image file: their view ids
    # come from the .sfm itself (stable), so they are appended to tell them apart.
    path_count = {}
    for v in views:
        path_count[rel(v["path"])] = path_count.get(rel(v["path"]), 0) + 1

    def key_of(v):
        p = rel(v["path"])
        return f"{p}#{int(v['viewId']):04d}" if path_count[p] > 1 else p

    view_key = {v["viewId"]: key_of(v) for v in views}
    # Ids that are not view ids are named after their first member (in view key order), so the comparison checks
    # which views share a pose, an intrinsic, a rig or an image group.
    intrinsic_users = {}
    rig_users = {}
    pose_users = {}
    group_users = {}
    for v in sorted(views, key=key_of):
        intrinsic_users.setdefault(v.get("intrinsicId"), key_of(v))
        if "rigId" in v:
            rig_users.setdefault(v["rigId"], key_of(v))
        if "poseId" in v:
            pose_users.setdefault(v["poseId"], key_of(v))
        if "imageGroupId" in v:
            group_users.setdefault(v["imageGroupId"], key_of(v))
    group_ids = {g.get("imageGroupId") for g in sfm.get("imageGroups", [])}

    def pose_label(pose_id):
        if pose_id in view_key:
            return view_key[pose_id]
        return "<pose of " + pose_users[pose_id] + ">"

    def group_label(group_id):
        if group_id not in group_ids:
            return "<missing group " + group_users[group_id] + ">"
        return group_users[group_id]

    out_views = {}
    for v in views:
        key = key_of(v)
        nv = {}
        for k, x in v.items():
            if k in ("viewId", "path"):
                continue
            if k == "poseId":
                nv["pose"] = pose_label(x)
            elif k == "imageGroupId":
                nv["imageGroup"] = group_label(x)
            elif k == "intrinsicId":
                nv["intrinsic"] = intrinsic_users.get(x)
            elif k == "rigId":
                nv["rig"] = rig_users.get(x)
            elif k == "metadata":
                nv["metadata"] = {mk: clean(mv) for mk, mv in x.items()
                                  if mk not in DROPPED_METADATA_KEYS and not mk.startswith(DROPPED_METADATA_PREFIXES)}
            else:
                nv[k] = clean(x)
        out_views[key] = nv

    def rig_label(match):
        return "_rig_" + str(rig_users.get(match.group(1), "<unknown rig>")) + "_"

    out_intrinsics = {}
    for i in sfm.get("intrinsics", []):
        label = intrinsic_users.get(i["intrinsicId"], "<unused " + str(len(out_intrinsics)) + ">")
        ni = {k: clean(x) for k, x in i.items() if k != "intrinsicId"}
        if "serialNumber" in ni:
            ni["serialNumber"] = re.sub(r"_rig_(\d+)_", rig_label, ni["serialNumber"])
        out_intrinsics[label] = ni

    out_rigs = sorted((len(r.get("subPoses", [])) for r in sfm.get("rigs", [])))
    out_groups = {}
    for g in sfm.get("imageGroups", []):
        label = group_users.get(g.get("imageGroupId"), "<unused " + str(len(out_groups)) + ">")
        out_groups[label] = {k: clean(x) for k, x in g.items() if k != "imageGroupId"}
    return {"views": out_views, "intrinsics": out_intrinsics, "rigs": out_rigs, "imageGroups": out_groups}


def _number(s):
    if not isinstance(s, str):
        return None
    try:
        return float(s)
    except ValueError:
        return None


def diff(expected, actual, path="", out=None):
    """List the differences between two normalized outputs (numbers compared with REL_TOL)."""
    out = [] if out is None else out
    if isinstance(expected, dict) and isinstance(actual, dict):
        for k in sorted(set(expected) | set(actual)):
            if k not in actual:
                out.append(f"{path}/{k}: missing (expected {json.dumps(expected[k])[:120]})")
            elif k not in expected:
                out.append(f"{path}/{k}: unexpected {json.dumps(actual[k])[:120]}")
            else:
                diff(expected[k], actual[k], f"{path}/{k}", out)
    elif isinstance(expected, list) and isinstance(actual, list) and len(expected) == len(actual):
        for i, (e, a) in enumerate(zip(expected, actual)):
            diff(e, a, f"{path}[{i}]", out)
    elif expected != actual:
        e, a = _number(expected), _number(actual)
        if e is not None and a is not None and (math.isclose(e, a, rel_tol=REL_TOL, abs_tol=1e-12)
                                               or (math.isnan(e) and math.isnan(a))):
            return out
        out.append(f"{path}: expected {json.dumps(expected)[:120]}, got {json.dumps(actual)[:120]}")
    return out


# ---------------------------------------------------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------------------------------------------------

def run_case(case, binary, data, expected_dir, update, keep, timeout):
    """Return a list of error strings (empty on success)."""
    errors = []
    with tempfile.TemporaryDirectory(prefix=f"imagedb_{case['name']}_") as tmp:
        tmp = Path(tmp)
        out = tmp / "out"
        out.mkdir()
        args = prepare_args(case, data, out, tmp)
        env = dict(os.environ)
        env.pop("ALICEVISION_ROOT", None)
        env.pop("ALICEVISION_SENSOR_DB", None)  # the cases choose the sensor database
        env.pop("ALICEVISION_LOG_LEVEL", None)  # would hide the expected warnings
        env["LC_ALL"] = "C"                     # system error messages in English
        env.update({k: substitute(v, data, out) for k, v in case["env"].items()})
        try:
            proc = subprocess.run([str(binary)] + args, env=env, cwd=tmp, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, timeout=timeout,
                                  preexec_fn=_no_core_dump if os.name == "posix" else None)
            log = proc.stdout.decode("utf-8", "replace")
            status = classify(proc.returncode)
        except subprocess.TimeoutExpired:
            return [f"timeout after {timeout}s"]

        if keep:
            kdir = Path(keep) / case["name"]
            shutil.rmtree(kdir, ignore_errors=True)
            shutil.copytree(out, kdir)
            (kdir / "log.txt").write_text(log)

        if not expected_matches(case["expect"], status):
            tail = "\n".join(log.splitlines()[-15:])
            errors.append(f"exit status '{status}' (code {proc.returncode}), expected '{case['expect']}'\n{tail}")
        for needle in case["log"]:
            if needle not in log:
                errors.append(f"log does not contain '{needle}'")

        if case["expect"] == "ok" and status == "ok":
            sfm_path = output_path(args)
            if sfm_path is None or not sfm_path.exists():
                errors.append(f"no output file {sfm_path}")
                return errors
            if sfm_path.suffix not in (".sfm", ".json"):
                return errors  # other formats (ply, baf, abc, usd): only checked to be written
            actual = normalize(json.loads(sfm_path.read_text()), data)
            ref = expected_dir / f"{case['name']}.json"
            if update:
                ref.parent.mkdir(parents=True, exist_ok=True)
                ref.write_text(json.dumps(actual, indent=1, sort_keys=True) + "\n")
            elif not ref.is_file():
                errors.append(f"no reference {ref} (run with --update)")
            else:
                differences = diff(json.loads(ref.read_text()), actual)
                if differences:
                    errors.append("output differs:\n  " + "\n  ".join(differences[:40]))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", required=True, help="aliceVision_cameraInit executable")
    parser.add_argument("--data", default=str(ROOT / "data"))
    parser.add_argument("--cases", default=str(ROOT / "cases.json"))
    parser.add_argument("--expected", default=str(ROOT / "expected"))
    parser.add_argument("--update", action="store_true", help="re-record expected/ from the current binary")
    parser.add_argument("--only", action="append", default=[], help="run cases whose name contains this text")
    parser.add_argument("--skip", action="append", default=[],
                        help="skip this case (e.g. output_format_abc in a build without Alembic)")
    parser.add_argument("--keep", help="copy each case output and log into this folder")
    parser.add_argument("--timeout", type=int, default=120)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    binary = Path(args.bin).resolve()  # the cases run in temporary folders: a relative path would not be found
    if not binary.is_file():
        logger.error("no executable %s", binary)
        return 1
    data = Path(args.data).resolve()
    cases = json.loads(Path(args.cases).read_text())
    if args.only:
        cases = [c for c in cases if any(o in c["name"] for o in args.only)]
    skipped = [c["name"] for c in cases
               if c["name"] in args.skip or (c.get("platform") and not sys.platform.startswith(c["platform"]))]
    cases = [c for c in cases if c["name"] not in skipped]
    if skipped:
        logger.info("skipped: %s", " ".join(skipped))
    failed = []
    for case in cases:
        errors = run_case(case, binary, data, Path(args.expected), args.update, args.keep, args.timeout)
        if errors:
            failed.append(case["name"])
            logger.error("FAIL %s\n  %s", case["name"], "\n  ".join(e.replace("\n", "\n  ") for e in errors))
        else:
            logger.info("ok   %s", case["name"])
    logger.info("%d/%d cases passed", len(cases) - len(failed), len(cases))
    if failed:
        logger.error("failed: %s", " ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
