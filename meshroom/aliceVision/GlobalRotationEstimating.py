__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class GlobalRotationEstimating(desc.AVCommandLineNode):
    """
Estimate global camera rotations from relative rotation measurements between image pairs.

As the first step of the Global SfM pipeline, this node aggregates all pairwise relative
rotations (estimated during feature matching and geometric filtering) into a consistent
set of absolute orientations for all cameras. It uses a robust rotation averaging method
to handle outlier relative rotations and recover the global rotation of every view.
"""

    commandLine = "aliceVision_globalRotationEstimating {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Sparse Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
            value="",
        ),
        desc.ChoiceParam(
            name="rotationAveragingMethod",
            label="Rotation Averaging Method",
            description="Method for rotation averaging:\n"
                        " - L1 minimization\n"
                        " - L2 minimization",
            values=["L1_minimization", "L2_minimization"],
            value="L2_minimization",
        ),
        desc.FloatParam(
            name="angularTolerance",
            label="Angular Tolerance",
            description="Angular (in degrees) tolerance for a given triplet.",
            value=5.0,
            range=(0.0, 180.0, 1.0),
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
        ),
    ]
