__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class MeshDecimate(desc.AVCommandLineNode):
    """
Reduce the polygon count of a 3D mesh while preserving its overall shape.

High-resolution meshes produced by multi-view reconstruction can contain millions of
triangles, which may be prohibitively large for real-time rendering or downstream
processing. This node applies a simplification algorithm to reduce the number of faces
by a configurable factor, or to a target face count, while minimising the geometric
error introduced by the simplification.
"""

    commandLine = "aliceVision_meshDecimate {allParams}"
    cpu = desc.Level.NORMAL
    ram = desc.Level.NORMAL

    category = "Mesh Post-Processing"
    inputs = [
        desc.File(
            name="input",
            label="Mesh",
            description="Input mesh in the OBJ format.",
            value="",
        ),
        desc.FloatParam(
            name="simplificationFactor",
            label="Simplification Factor",
            description="Simplification factor for the decimation.",
            value=0.5,
            range=(0.0, 1.0, 0.01),
        ),
        desc.IntParam(
            name="nbVertices",
            label="Fixed Number of Vertices",
            description="Fixed number of output vertices.",
            value=0,
            range=(0, 1000000, 1),
        ),
        desc.IntParam(
            name="minVertices",
            label="Min Vertices",
            description="Minimum number of output vertices.",
            value=0,
            range=(0, 1000000, 1),
        ),
        desc.IntParam(
            name="maxVertices",
            label="Max Vertices",
            description="Maximum number of output vertices.",
            value=0,
            range=(0, 1000000, 1),
        ),
        desc.BoolParam(
            name="flipNormals",
            label="Flip Normals",
            description="Option to flip face normals.\n"
                        "It can be needed as it depends on the vertices order in triangles\n"
                        "and the convention changes from one software to another.",
            value=False,
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
            description="Output mesh in the OBJ file format.",
            value="{nodeCacheFolder}/mesh.obj",
        ),
    ]
