__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class MeshRemoveUnseenFaces(desc.AVCommandLineNode):
    """
Remove triangles from the mesh that are not visible from any camera.

After dense meshing, some triangles may correspond to surfaces that are never directly
observed in any input image (e.g., the underside of an object or geometry in occluded
regions). This node removes such triangles by checking the visibility of each vertex or
face against the known camera frusta and the reconstructed depth information, producing
a cleaner mesh that contains only observable geometry.
"""

    commandLine = "aliceVision_meshRemoveUnseenFaces {allParams}"

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.NORMAL

    category = "Dense Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="inputMesh",
            label="Mesh",
            description="Input Mesh file.",
            value="",
        ),
        desc.ChoiceParam(
            name="outputMeshFileType",
            label="Mesh Type",
            description="File type for the output mesh.",
            value="obj",
            values=["gltf", "obj", "fbx", "stl"],
            commandLineGroup="",
        ),
        desc.IntParam(
            name="minObservations",
            label="Min Observations",
            description="Minimal number of observations to keep a vertex.",
            value=1,
            range=(0, 5, 1),
        ),
        desc.IntParam(
            name="minVertices",
            label="Min Vertices to Remove a Triangle",
            description="Minimal number of killed vertices in a triangle to remove the triangle.",
            value=3,
            range=(1, 3, 1),
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
            name="outputMesh",
            label="Mesh",
            description="Output mesh file.",
            value="{nodeCacheFolder}/mesh.{outputMeshFileTypeValue}",
        ),
    ]
