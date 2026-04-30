__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMTriangulating(desc.AVCommandLineNode):
    commandLine = "aliceVision_sfmTriangulating {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Sparse Reconstruction"
    documentation = """
This node performs keypoint triangulation on its input data.
Contrary to the StructureFromMotion node, this node does not infer the camera poses, therefore they must be given in the SfMData input.
"""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file. Must contain the camera calibration.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.IntParam(
            name="minNumberOfObservationsForTriangulation",
            label="Min Observations For Triangulation",
            description="Minimum number of observations to triangulate a point.\n"
                        "Setting it to 3 (or more) reduces drastically the noise in the point cloud.",
            value=2,
            range=(2, 10, 1),
            advanced=True,
        ),
        desc.FloatParam(
            name="minAngleForTriangulation",
            label="Min Angle For Triangulation",
            description="Minimum angle for triangulation.",
            value=3.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="maxTriangulationError",
            label="Max Triangulation Error",
            description="Maximum reprojection error in the triangulation process (in pixels).",
            value=8.0,
            range=(0.1, 10.0, 0.1),
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
            label="SfMData",
            description="Path to the output SfMData file.",
            value="{nodeCacheFolder}/sfm.abc",
        )
    ]
