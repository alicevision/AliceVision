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


# Returns a tuple of
# (1) Whether op was successful
# (2) Resolved paths
# (3) Tuple of reasons and unresolved paths (must be empty on success)
# (4) Tuple of conflicting paths per dependency
# (5) The resolved rpaths to pass through to subdependencies
def try_and_match_deps(
    input: tuple[Path, list[Path], list[Path]],
    globalCache,
    additionalLookupPaths: Optional[set[Path]] = None,
) -> tuple[
    bool,
    list[Path],
    list[tuple[list[str], Path]],
    list[tuple[list[Path], Path]],
    set[Path],
]:
    possibleReasonsForErr: list[str] = []

    # If the input file has no extension, we assume it is an executable and resolve any @executable_paths
    isExecutable = False
    if input[0].suffix == "":
        isExecutable = True

    # We properly create paths from @executable_path (if possible), @loader_path and relative paths
    for i, rpath in enumerate(input[2]):
        if "@executable_path" in rpath.parts:
            if isExecutable:
                input[2][i] = (
                    input[0].parent.parent.joinpath(Path(*rpath.parts[1:])).resolve()
                )
            else:
                possibleReasonsForErr.append(
                    f"Could not resolve {rpath}! Input file is not an executable: {input[0]}."
                )
        elif "@loader_path" in rpath.parts:
            input[2][i] = input[0].parent.joinpath(Path(*rpath.parts[1:])).resolve()
        elif not rpath.is_absolute():
            input[2][i] = input[0].parent.joinpath(rpath).resolve()

    # Remove duplicates
    uniqueLookupPaths = set(input[2])
    # Add additonal lookup paths from parents
    if additionalLookupPaths:
        uniqueLookupPaths.update(additionalLookupPaths)

    # Try to resolve the required libraries with the available rpaths
    resolvedPaths: list[Path] = []
    unresolvedPaths: list[tuple[list[str], Path]] = []
    conflicitingPaths: list[tuple[list[Path], Path]] = []
    for dep in input[1]:
        isCached = False
        for macho in globalCache.items():
            if dep.stem == macho[0].stem and get_archs(input[0]).issubset(macho[1]):
                isCached = True
                break
        if isCached:
            continue
        # Filter system libraries
        if "/usr/lib" in str(dep) or "/System" in str(dep):
            continue
        # If not @rpath prefixed, check if absolute
        elif "@rpath" not in dep.parts and dep.is_absolute():
            if dep.exists():
                if check_arch_match(input[0], dep):
                    resolvedPaths.append(dep)
                    continue
                else:
                    print(
                        f"[ WARN ] Resolved dependency exists, but the architectures do not match: Dependant: {input[0]}, resolved dependency: {dep}."
                    )
                    continue
            else:
                unresolvedPaths.append(
                    (["Absolute path of dependency does not exist!"], dep)
                )
                continue
        elif "@rpath" in dep.parts:
            depRpathStripped = Path(*dep.parts[1:])
            resolvedPathsInner: list[Path] = []
            for lookupPath in uniqueLookupPaths:
                if lookupPath.joinpath(depRpathStripped).exists():
                    if check_arch_match(
                        input[0], lookupPath.joinpath(depRpathStripped)
                    ):
                        resolvedPathsInner.append(lookupPath.joinpath(depRpathStripped))
                        continue
                    else:
                        print(
                            f"[ WARN ] Resolved dependency exists, but the architectures do not match: Dependant: {input[0]}, resolved dependency: {lookupPath.joinpath(depRpathStripped)}."
                        )
                        continue
            if len(resolvedPathsInner) == 0:
                reasons: list[str] = [
                    f"No exctracted rpaths were able to resolve the dependency! Required by: {input[0]}."
                ]
                if not isExecutable:
                    reasons += possibleReasonsForErr
                unresolvedPaths.append((reasons, dep))
            elif len(resolvedPathsInner) > 1:
                conflicitingPaths.append((resolvedPathsInner, dep))
                resolvedPaths.append(resolvedPathsInner[0])
            else:
                resolvedPaths += resolvedPathsInner
            continue
        elif not dep.is_absolute():
            resolvedPathsInner: list[Path] = []
            for lookupPath in uniqueLookupPaths:
                if lookupPath.joinpath(dep).exists():
                    if check_arch_match(input[0], lookupPath.joinpath(dep)):
                        resolvedPathsInner.append(lookupPath.joinpath(dep))
                        continue
                    else:
                        print(
                            f"[ WARN ] Resolved dependency exists, but the architectures do not match: Dependant: {input[0]}, resolved dependency: {lookupPath.joinpath(dep)}."
                        )
                        continue
            if len(resolvedPathsInner) == 0:
                unresolvedPaths.append(
                    (
                        [
                            "The relative path of the dependency did not resolve to an existing dependency!"
                        ],
                        dep,
                    )
                )
            elif len(resolvedPathsInner) > 1:
                conflicitingPaths.append((resolvedPathsInner, dep))
                resolvedPaths.append(resolvedPathsInner[0])
            else:
                resolvedPaths += resolvedPathsInner
            continue
        else:
            unresolvedPaths.append(
                (["Encountered unknown dependency path scheme!"], dep)
            )

    for resolvedPath in resolvedPaths:
        globalCache[resolvedPath] = get_archs(resolvedPath)
    return (
        len(unresolvedPaths) == 0,
        resolvedPaths,
        unresolvedPaths,
        conflicitingPaths,
        uniqueLookupPaths,
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


def entry():
    # Create the parser
    parser = argparse.ArgumentParser(
        description="Make a self-contained AliceVision bundle on Darwin"
    )

    # Optional output directory
    _ = parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path.cwd().joinpath("bundle"),
        help="Output directory",
    )

    # Positional arguments: arbitrary number of file paths
    _ = parser.add_argument(
        "input_files", type=Path, nargs="+", help="Input files to process"
    )

    # Parse args
    args = parser.parse_args()

    # Access them
    outputDir: Path = args.output
    inputFiles: list[Path] = args.input_files

    # Log Info about input
    print(f"[ INFO ] Placing bundle at: {outputDir}.")
    print("[ INFO ] Attempting for files:")
    for inputFile in inputFiles:
        print("[ INFO ] \t" + str(inputFile))
    print("\n[ INFO ] ### Patience... ###\n")

    # Get all binaries
    inputMachOBins: list[Path] = []
    for file in inputFiles:
        fileCmd = subprocess.run(
            ["file", file], universal_newlines=True, stdout=subprocess.PIPE
        )
        if "Mach-O" in fileCmd.stdout:
            inputMachOBins.append(file)

    # Get all libraries and Frameworks
    inputMachOLibs: list[Path] = []
    for file in inputFiles:
        if "dylib" in file.suffix:
            fileCmd = subprocess.run(
                ["file", file], universal_newlines=True, stdout=subprocess.PIPE
            )
            if "Mach-O" in fileCmd.stdout:
                inputMachOLibs.append(file)
        elif "framework" in file.suffix:
            fileCmd = subprocess.run(
                ["file", file.joinpath(file.stem)],
                universal_newlines=True,
                stdout=subprocess.PIPE,
            )
            if "Mach-O" in fileCmd.stdout:
                inputMachOLibs.append(file)
        else:
            continue

    # Create concurrent interpreters
    concurrentPool = Pool(cpu_count())

    print("[ INFO ] (1 / 5) Looking up required dependencies and embedded rpaths...")

    # Extract required dependencies and rpaths
    depsAndRpathsPerInput: list[tuple[Path, list[Path], list[Path]]] = (
        concurrentPool.map(get_deps_and_rpaths, inputMachOLibs + inputMachOBins)
    )

    print("[ INFO ] (2 / 5) Attempting to resolve all dependencies...")

    # Resolve per input and subdependency
    manager = Manager()
    globalCache: DictProxy[Path, set[str]] = manager.dict()
    resolveFunc = partial(traverse_deps_and_resolve, globalCache=globalCache)
    resolverResultPerInput: list[
        tuple[
            bool,
            list[Path],
            list[tuple[list[str], Path]],
            list[tuple[list[Path], Path]],
        ]
    ] = concurrentPool.map(resolveFunc, depsAndRpathsPerInput)

    # Check for Results
    isSuccessful = True
    for result in resolverResultPerInput:
        if not result[0]:
            print("[ ERROR ] An error occured during the resolving process:")
            for unresolved in result[2]:
                print(
                    f"[ ERROR ] \tDependency: {str(unresolved[1])}, failed with: {str(unresolved[0])}"
                )
            isSuccessful = False
        if len(result[3]) != 0:
            for conflictingDep in result[3]:
                print(
                    f"[ WARN ] Multiple paths were found to resolve {str(conflictingDep[1])}:"
                )
                for conflictingPath in conflictingDep[0]:
                    print(f"[ WARN ] \tFound suitable: {str(conflictingPath)}")

    if not isSuccessful:
        print("[ ERROR ] Errors occured! Refusing to build bundle.")
        exit(-1)

    print("[ INFO ] (3 / 5) Making bundle structure...")

    # Create the output directory
    shutil.rmtree(outputDir, ignore_errors=True)
    outputDir.mkdir(parents=True, exist_ok=True)
    (outputDir / "lib").mkdir(parents=True, exist_ok=True)
    (outputDir / "bin").mkdir(parents=True, exist_ok=True)

    # Copy input files
    # Determine if they are dylibs/Frameworks or executables
    inputBins: set[Path] = set()
    inputLibs: set[Path] = set()
    for inputFile in inputFiles:
        if inputFile.suffix == "":
            inputBins.add(inputFile)
        else:
            inputLibs.add(inputFile)

    # Copy bins
    dstBin = partial(copy_safe, dst_dir=outputDir / "bin")
    destBins = set(list(concurrentPool.map(dstBin, inputBins)))

    # Copy libs
    dstLib = partial(copy_safe, dst_dir=outputDir / "lib")
    destLibs = set(list(concurrentPool.map(dstLib, inputLibs)))

    print("[ INFO ] (4 / 5) Copying required files...")

    # Create set for files to copy
    filesToCopy: set[Path] = set()
    # Must handle special cases of Frameworks
    for result in resolverResultPerInput:
        for resolvedPath in result[1]:
            if resolvedPath.suffix == "":
                # We want to get the actual .framework folder.
                # Means we call parent until the suffix is .framework
                frameworkFolder = resolvedPath
                while not frameworkFolder.suffix == ".framework":
                    frameworkFolder = frameworkFolder.parent
                filesToCopy.add(frameworkFolder)
            else:
                filesToCopy.add(resolvedPath)

    # Copy all into new bundle
    dstResolvedLibs = set(list(concurrentPool.map(dstLib, filesToCopy)))

    # Create destination list
    allDstFiles = dstResolvedLibs.union(destLibs).union(destBins)

    print("[ INFO ] (5 / 5) Fixing up copied files...")

    # Fixup all destination files
    successList: list[bool] = concurrentPool.map(
        fixup_macho_with_predefined_rpaths, allDstFiles
    )

    # Done
    if False in successList:
        print("[ ERROR ] Errors occured during fixup. Bundle will be unfunctional.")
        exit(-1)
    else:
        print(
            f"\n[ INFO ] ### Successfully created self-contained bundle at {outputDir.resolve()}. ###"
        )


# Only launch when called directly
if __name__ == "__main__":
    entry()
