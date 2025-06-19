import os

# Windows only
if hasattr(os, 'add_dll_directory'):
    paths = os.environ['PATH'].split(os.pathsep)
    for p in paths:
        if not os.path.isdir(p):
            continue
        os.add_dll_directory(p)
