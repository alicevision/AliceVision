__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class PanoramaRefining(desc.AVCommandLineNode):
    """
Refine panorama camera rotations using bundle adjustment.

This node performs a joint optimisation (bundle adjustment) over all camera rotations and the
shared focal length to minimise reprojection errors across all feature matches. The
refined rotations produce a more accurate panorama with fewer stitching artefacts,
especially in scenes with strong parallax or lens distortion.
"""

    commandLine = "aliceVision_panoramaRefining {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Panorama HDR"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),      
        desc.BoolParam(
            name="intermediateRefineWithFocal",
            label="Intermediate Refine: Focal",
            description="Intermediate refine with rotation and focal length only.",
            value=False,
            advanced=True,
        ),
        desc.BoolParam(
            name="intermediateRefineWithFocalDist",
            label="Intermediate Refine: Focal And Distortion",
            description="Intermediate refine with rotation, focal length and distortion.",
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
            label="SfM File",
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/panorama.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        ),
    ]
