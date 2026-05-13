__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class NodalSfM(desc.AVCommandLineNode):
    """
Perform Structure-from-Motion for scenes captured with a camera undergoing pure rotation.

Unlike standard SfM pipelines that require translation between views to triangulate 3D
points, this node is designed for panoramic capture setups where the camera rotates around
a single nodal point (no parallax). It estimates the relative rotation between each pair
of images and recovers the global camera orientations. The output is an SfMData with
calibrated rotations but no translational component, suitable for panorama stitching.
"""

    commandLine = "aliceVision_nodalSfM {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Sparse Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Input tracks file.",
            value="",
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
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
            value="{nodeCacheFolder}/sfm.abc",
        ),
    ]
