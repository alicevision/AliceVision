import os

# Windows only
if hasattr(os, "add_dll_directory"):
    import pathlib
    pyavFolder = pathlib.Path(__file__).resolve().parent
    os.add_dll_directory(pyavFolder)

    avRoot = os.environ.get("ALICEVISION_ROOT", "")
    if not avRoot:
        avRoot = pyavFolder.parent.parent.parent
    avBin = os.path.join(avRoot, "bin")

    os.add_dll_directory(avBin)

    libfolders = os.environ.get("ALICEVISION_LIBPATH", "").split(os.pathsep)
    for p in libfolders:
        if not os.path.isdir(p):
            continue
        os.add_dll_directory(p)

#Enforce loading order
from . import system
from . import geometry
from . import sfmData
from . import sfmDataIO

#initialize logger
system.initialize_alicevision_logger("info")
