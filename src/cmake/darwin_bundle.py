#
# Darwin Bundle
#
# Creates a standalone AliceVision bundle for use in Meshroom for Apple targets
#
# Usage: python3 darwin_bundle.py [-o <OUTPUT_DIR>] <INPUT_FILES>...
#

import argparse
import shutil
import subprocess

from functools import partial
from multiprocessing import Pool, cpu_count, Manager
from multiprocessing.managers import DictProxy
from pathlib import Path
from re import sub
from typing import Optional

TARGET_RPATHS: list[str] = [
    "@executable_path",
    "@executable_path/../lib",
    "@loader_path",
    "@loader_path/../lib",
]

# Returns a tuple of
# (1) The name of the Mach-O
# (2) The required dependencies
# (3) The available rpaths
def get_deps_and_rpaths(macho: Path) -> tuple[Path, list[Path], list[Path]]:
    # If Framework, we need to check the inner Mach-O
    oldPath = macho
    if "framework" in macho.suffix:
        macho = macho.joinpath(macho.stem)
    deps: list[Path] = []
    rpaths: list[Path] = []
    depsCmd = subprocess.run(
        ["otool", "-L", macho], universal_newlines=True, stdout=subprocess.PIPE
    )
    depsLines = depsCmd.stdout.splitlines()
    for line in depsLines[
        2:
    ]:  # Skip the first line (just info) and the second line (always denotes itself)
        deps.append(
            Path(sub(r"\(compatibility version [^)]+\)", "", line.strip()).strip())
        )  # Remove the compatibility stuff

    rpathCmd = subprocess.run(
        ["otool", "-l", macho], universal_newlines=True, stdout=subprocess.PIPE
    )
    rpathLines = iter(rpathCmd.stdout.splitlines())
    for line in rpathLines:
        if "LC_RPATH" in line.strip():
            _ = next(rpathLines, None)  # This is "cmdsize XX"
            rpaths.append(
                Path(
                    sub(
                        r"\(offset \d+\)",
                        "",
                        next(rpathLines, "").strip().removeprefix("path "),
                    ).strip()
                )
            )  # This is the rpath

    return (
        oldPath,
        deps,
        rpaths,
    )  # Return the old path, so we still have .framework (not .framework/Mach-O)

# Extracts the architectures of a Mach-O file
def get_archs(path: Path) -> set[str]:
    if ".framework" in path.suffix:
        path = path.joinpath(path.stem)
    result = subprocess.run(
        ["lipo", "-archs", str(path)],
        universal_newlines=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return set(result.stdout.strip().split())

# Checks if lh and rh share the same architecture
def check_arch_match(lh: Path, rh: Path) -> bool:
    lh_archs = get_archs(lh)
    rh_archs = get_archs(rh)
    # True if either is universal (more than one arch)
    # or if they share at least one arch
    if len(lh_archs) > 1 or len(rh_archs) > 1:
        return True
    return not lh_archs.isdisjoint(rh_archs)

def resolve_rpaths(
    macho: Path,
    rpaths: list[Path],
    is_executable: bool
) -> tuple[list[Path], list[str]]:
    resolved = []
    errors = []

    for rpath in rpaths:
        if "@executable_path" in rpath.parts:
            if is_executable:
                resolved.append(
                    macho.parent.parent.joinpath(*rpath.parts[1:]).resolve()
                )
            else:
                errors.append(f"Cannot resolve {rpath} (not an executable)")
        elif "@loader_path" in rpath.parts:
            resolved.append(
                macho.parent.joinpath(*rpath.parts[1:]).resolve()
            )
        elif not rpath.is_absolute():
            resolved.append(macho.parent.joinpath(rpath).resolve())
        else:
            resolved.append(rpath)

    return resolved, errors

def build_lookup_paths(
    rpaths: list[Path],
    additional: Optional[set[Path]]
) -> set[Path]:
    lookup = set(rpaths)
    if additional:
        lookup.update(additional)
    return lookup

def is_cached(dep: Path, macho: Path, cache) -> bool:
    for cached_path, archs in cache.items():
        if dep.stem == cached_path.stem and get_archs(macho).issubset(archs):
            return True
    return False

def resolve_dependency(
    dep: Path,
    macho: Path,
    lookup_paths: set[Path],
    is_executable: bool,
    rpath_errors: list[str],
) -> tuple[list[Path], list[str], list[list[Path]]]:
    resolved = []
    errors = []
    conflicts = []

    # Skip system libs
    if "/usr/lib" in str(dep) or "/System" in str(dep):
        return resolved, errors, conflicts

    # Absolute path
    if dep.is_absolute() and "@rpath" not in dep.parts:
        if dep.exists():
            if check_arch_match(macho, dep):
                resolved.append(dep)
            else:
                print(f"[ WARN ] Architecture mismatch: {dep}")
        else:
            errors.append("Absolute path does not exist")
        return resolved, errors, conflicts

    # @rpath case
    if "@rpath" in dep.parts:
        target = Path(*dep.parts[1:])
        matches = [
            p / target for p in lookup_paths
            if (p / target).exists() and check_arch_match(macho, p / target)
        ]

        if not matches:
            errors.append("Could not resolve via rpaths")
            if not is_executable:
                errors.extend(rpath_errors)
        elif len(matches) > 1:
            conflicts.append(matches)
            resolved.append(matches[0])
        else:
            resolved.extend(matches)

        return resolved, errors, conflicts

    # Relative path
    if not dep.is_absolute():
        matches = [
            p / dep for p in lookup_paths
            if (p / dep).exists() and check_arch_match(macho, p / dep)
        ]

        if not matches:
            errors.append("Relative path did not resolve")
        elif len(matches) > 1:
            conflicts.append(matches)
            resolved.append(matches[0])
        else:
            resolved.extend(matches)

        return resolved, errors, conflicts

    # Fallback
    errors.append("Unknown dependency scheme")
    return resolved, errors, conflicts

# Returns a tuple of
# (1) Whether op was successful
# (2) Resolved paths
# (3) Tuple of reasons and unresolved paths (must be empty on success)
# (4) Tuple of conflicting paths per dependency
# (5) The resolved rpaths to pass through to subdependencies
def try_and_match_deps(input, globalCache, additionalLookupPaths=None):
    macho, deps, rpaths = input
    is_executable = macho.suffix == ""

    resolved_rpaths, rpath_errors = resolve_rpaths(
        macho, rpaths, is_executable
    )

    lookup_paths = build_lookup_paths(resolved_rpaths, additionalLookupPaths)

    resolved = []
    unresolved = []
    conflicts = []

    for dep in deps:
        if is_cached(dep, macho, globalCache):
            continue

        r, e, c = resolve_dependency(
            dep, macho, lookup_paths, is_executable, rpath_errors
        )

        resolved.extend(r)
        conflicts.extend([(paths, dep) for paths in c])

        if e:
            unresolved.append((e, dep))

    for path in resolved:
        globalCache[path] = get_archs(path)

    return (
        len(unresolved) == 0,
        resolved,
        unresolved,
        conflicts,
        lookup_paths,
    )

def traverse_deps_and_resolve(
    input: tuple[Path, list[Path], list[Path]], globalCache
) -> tuple[
    bool, list[Path], list[tuple[list[str], Path]], list[tuple[list[Path], Path]]
]:
    # Initial state
    successTop, resolvedTop, unresolvedTop, conflictingTop, lookupPathsCombined = (
        try_and_match_deps(input, globalCache)
    )

    # Use sets for quick membership tests
    resolved_set = set(resolvedTop)
    processed: set[Path] = set()  # things we have already pulled deps for
    queue: list[Path] = list(resolvedTop)  # things we still need to process

    # BFS-style traversal through dependencies
    while queue:
        subDep = queue.pop(0)
        if subDep in processed:
            continue
        processed.add(subDep)

        ok, resolved, unresolved, conflicting, uniqueLookupPaths = try_and_match_deps(
            get_deps_and_rpaths(subDep), globalCache, lookupPathsCombined
        )

        # Update overall success flag
        if not ok:
            successTop = False

        # Update lookup paths
        lookupPathsCombined.update(uniqueLookupPaths)

        # Add newly resolved dependencies:
        for newDep in resolved:
            if newDep not in resolved_set:
                resolved_set.add(newDep)
                resolvedTop.append(newDep)
                queue.append(newDep)

        # Update unresolved and conflicting lists
        unresolvedTop += unresolved
        conflictingTop += conflicting

    return (successTop, resolvedTop, unresolvedTop, conflictingTop)

def copy_safe(src: Path, dst_dir: Path) -> Path:
    dst = dst_dir / src.name

    if dst.exists():
        return dst

    if src.is_dir():
        _ = shutil.copytree(
            src,
            dst,
            symlinks=True,
        )
    elif src.is_symlink():
        # Resolve the target of the symlink
        target = src.resolve()
        # Copy the target file first
        if not (dst_dir / target.name).exists():
            _ = shutil.copy2(target, dst_dir / target.name)
        # Recreate the symlink in the target dir
        dst.symlink_to(target.name)
    else:
        # Regular file
        _ = shutil.copy2(src, dst)

    return dst

def fixup_macho_with_predefined_rpaths(macho: Path) -> bool:
    success = True

    # Special case Framework
    if macho.suffix == ".framework":
        machoInner = macho.joinpath(macho.stem)
    else:
        machoInner = macho

    rpaths: set[Path] = set()
    # Extract existing rpaths
    rpathCmd = subprocess.run(
        ["otool", "-l", machoInner],
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    rpathLines = iter(rpathCmd.stdout.splitlines())
    for line in rpathLines:
        if "LC_RPATH" in line.strip():
            _ = next(rpathLines, None)  # This is "cmdsize XX"
            rpaths.add(
                Path(
                    sub(
                        r"\(offset \d+\)",
                        "",
                        next(rpathLines, "").strip().removeprefix("path "),
                    ).strip()
                )
            )  # This is the rpath

    # Remove existing rpaths
    for rpath in rpaths:
        _ = subprocess.run(
            ["install_name_tool", "-delete_rpath", rpath, machoInner],
            universal_newlines=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # Add predefined set of target rpaths
    for rpath in TARGET_RPATHS:
        _ = subprocess.run(
            ["install_name_tool", "-add_rpath", rpath, machoInner],
            universal_newlines=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # Re-sign the binary with ad-hoc signature
    # Otherwise it will be terminated by SIGABRT
    _ = subprocess.run(
        ["codesign", "--force", "--deep", "--sign", "-", macho],
        universal_newlines=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    return success

def parse_args():
    parser = argparse.ArgumentParser(
        description="Make a self-contained AliceVision bundle on Darwin"
    )

    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=Path.cwd() / "bundle",
        help="Output directory",
    )

    parser.add_argument(
        "input_files",
        type=Path,
        nargs="+",
        help="Input files to process",
    )

    return parser.parse_args()

def log_inputs(output_dir: Path, input_files: list[Path]):
    print(f"[ INFO ] Placing bundle at: {output_dir}.")
    print("[ INFO ] Attempting for files:")
    for f in input_files:
        print(f"[ INFO ] \t{f}")
    print("\n[ INFO ] ### Patience... ###\n")

def is_macho(file: Path) -> bool:
    result = subprocess.run(
        ["file", file],
        text=True,
        stdout=subprocess.PIPE
    )
    return "Mach-O" in result.stdout

def classify_inputs(input_files: list[Path]) -> tuple[list[Path], list[Path]]:
    bins = []
    libs = []

    for f in input_files:
        if not is_macho(f):
            continue

        if f.suffix == "":
            bins.append(f)
        elif f.suffix == ".dylib":
            libs.append(f)
        elif f.suffix == ".framework":
            inner = f / f.stem
            if is_macho(inner):
                libs.append(f)

    return bins, libs

def extract_dependencies(files: list[Path]):
    with Pool(cpu_count()) as pool:
        return pool.map(get_deps_and_rpaths, files)

def resolve_dependencies(deps_info):
    with Pool(cpu_count()) as pool:
        manager = Manager()
        cache = manager.dict()

        resolver = partial(traverse_deps_and_resolve, globalCache=cache)
        return pool.map(resolver, deps_info)

def ensure_success(results):
    success = True

    for ok, _, unresolved, conflicts in results:
        if not ok:
            print("[ ERROR ] Dependency resolution failed:")
            for reasons, dep in unresolved:
                print(f"[ ERROR ] \t{dep}: {reasons}")
            success = False

        for paths, dep in conflicts:
            print(f"[ WARN ] Multiple candidates for {dep}:")
            for p in paths:
                print(f"[ WARN ] \t{p}")

    if not success:
        print("[ ERROR ] Aborting bundle creation.")
        exit(-1)

def create_bundle_structure(output_dir: Path):
    shutil.rmtree(output_dir, ignore_errors=True)
    (output_dir / "bin").mkdir(parents=True)
    (output_dir / "lib").mkdir(parents=True)

    return {
        "root": output_dir,
        "bin": output_dir / "bin",
        "lib": output_dir / "lib",
    }

def split_inputs(input_files):
    bins = {f for f in input_files if f.suffix == ""}
    libs = set(input_files) - bins
    return bins, libs

def collect_dependencies(results):
    files = set()

    for _, resolved, _, _ in results:
        for path in resolved:
            if path.suffix == "":
                while path.suffix != ".framework":
                    path = path.parent
            files.add(path)

    return files

def copy_all_files(bundle_paths, input_files, results):
    bins, libs = split_inputs(input_files)
    deps = collect_dependencies(results)

    with Pool(cpu_count()) as pool:
        copy_bin = partial(copy_safe, dst_dir=bundle_paths["bin"])
        copy_lib = partial(copy_safe, dst_dir=bundle_paths["lib"])

        dest_bins = set(pool.map(copy_bin, bins))
        dest_libs = set(pool.map(copy_lib, libs))
        dest_deps = set(pool.map(copy_lib, deps))

    return dest_bins | dest_libs | dest_deps

def fixup_binaries(files: set[Path]):
    with Pool(cpu_count()) as pool:
        results = pool.map(fixup_macho_with_predefined_rpaths, files)

    if not all(results):
        print("[ ERROR ] Fixup failed.")
        exit(-1)

def entry():
    args = parse_args()

    log_inputs(args.output, args.input_files)

    bins, libs = classify_inputs(args.input_files)

    print("[ INFO ] (1 / 5) Extracting dependencies...")
    deps_info = extract_dependencies(bins + libs)

    print("[ INFO ] (2 / 5) Resolving dependencies...")
    results = resolve_dependencies(deps_info)

    ensure_success(results)

    print("[ INFO ] (3 / 5) Creating bundle...")
    bundle_paths = create_bundle_structure(args.output)

    print("[ INFO ] (4 / 5) Copying files...")
    copied_files = copy_all_files(bundle_paths, args.input_files, results)

    print("[ INFO ] (5 / 5) Fixing binaries...")
    fixup_binaries(copied_files)

    print(f"\n[ INFO ] Bundle created at {args.output.resolve()}")

# Only launch when called directly
if __name__ == "__main__":
    entry()
