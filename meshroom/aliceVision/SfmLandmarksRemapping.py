__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMLandmarksRemapping(desc.AVCommandLineNode):
    """
Synchronize landmark identifiers in an SfMData scene with their corresponding track identifiers.

In AliceVision, each 3D landmark is created from a feature track and should share the same
identifier as that track. However, after certain operations on the tracks (e.g., merging or
filtering), the track IDs may have changed while the landmark IDs in the SfMData remain
outdated. This node updates each landmark's ID to match its associated track ID, restoring
the expected correspondence that many downstream nodes rely on.
"""

    commandLine = "aliceVision_sfmLandmarksRemapping {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
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
            value="{nodeCacheFolder}/sfmData.sfm",
        )
    ]
