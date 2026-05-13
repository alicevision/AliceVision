__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ConvertMesh(desc.AVCommandLineNode):
    """
Convert a 3D mesh from one file format to another.

Supported formats include OBJ, FBX, glTF, GLB, STL and PLY. The node can optionally flip
face normals (which may be required depending on the winding-order convention used by the
target application) and copy associated texture image files to the output folder.
"""

    commandLine = "aliceVision_convertMesh {allParams}"
    category = "Utils"
    inputs = [
        desc.File(
            name="inputMesh",
            label="Input Mesh",
            description="Input mesh (*.obj, *.fbx, *.gltf, *.glb, *.stl, *.ply).",
            value="",
        ),
        desc.ChoiceParam(
            name="outputMeshFileType",
            label="Output File Type",
            description="Output mesh format (*.obj, *.fbx, *.gltf, *.glb, *.stl, *.ply).",
            value="obj",
            values=["obj", "fbx", "gltf", "glb", "stl", "ply"],
            commandLineGroup="",
        ),
        desc.BoolParam(
            name="flipNormals",
            label="Flip Normals",
            description="Flip face normals. It can be needed as it depends on the vertices order "
                        "in triangles and the convention changes from one software to another.",
            value=False,
            advanced=True,
        ),
        desc.BoolParam(
            name="copyTextures",
            label="Copy Textures",
            description="Copy input mesh texture files to the output mesh folder.",
            value=True,
            advanced=True,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="Mesh",
            description="Output mesh (*.obj, *.fbx, *.gltf, *.glb, *.stl, *.ply).",
            value="{nodeCacheFolder}/mesh.{outputMeshFileTypeValue}",
        ),
    ]
