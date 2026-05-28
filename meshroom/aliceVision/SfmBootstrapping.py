__version__ = "4.2"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMBootStrapping(desc.AVCommandLineNode):
    """
Initialize the incremental Structure-from-Motion reconstruction by selecting the best initial image pair.

This node identifies the optimal pair of images from which to begin the reconstruction.
A good initial pair has sufficient visual overlap, a wide baseline, and enough matched features
to compute a reliable fundamental or homography matrix. Several bootstrapping methods are available:

 - **classic**: Uses epipolar geometry to select the best initial pair based on feature matches.
 - **mesh**: Constrains the initial pair selection using a reference 3D mesh.
 - **mesh_single**: Like mesh, but does not require visual parallax between the two views.
 - **depth**: Uses depth map information to initialize the reconstruction.

The output is a partially initialized SfMData with the first two cameras localized.
"""

    commandLine = "aliceVision_sfmBootstrapping {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Sparse Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.ChoiceParam(
            name="method",
            label="Method",
            description="Bootstrapping method: classic (epipolar geometry), mesh (3D mesh constraints), mesh_single (mesh without visual parallax), or depth (depth map information).",
            values=["classic", "mesh", "mesh_single", "depth"],
            value="classic",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.File(
            name="meshFilename",
            label="Mesh File",
            description="Mesh file (*.obj).",
            value="",
            enabled=lambda node: node.method.value.startswith("mesh")
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
            value="",
        ),
        desc.FloatParam(
            name="minAngleSoftInitialPair",
            label="Min Angle Initial Pair (Soft)",
            description="Pair score is downgraded heavily if angle is under this value",
            value=5.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="minAngleHardInitialPair",
            label="Min Angle Initial Pair (Hard)",
            description="Minimum angle for the initial pair validation.",
            value=1.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="maxAngleInitialPair",
            label="Max Angle Initial Pair",
            description="Maximum angle for the initial pair.",
            value=40.0,
            range=(0.1, 60.0, 0.1),
            advanced=True,
        ),
        desc.File(
            name="initialPairA",
            label="Initial Pair A",
            description="View ID of the first image.",
            value="",
        ),
        desc.File(
            name="initialPairB",
            label="Initial Pair B",
            description="View ID of the second image.",
            value="",
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
            value="{nodeCacheFolder}/bootstrap.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
