__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMLandmarksRemapping(desc.AVCommandLineNode):
    commandLine = "aliceVision_sfmLandmarksRemapping {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    documentation = """
    A landmark is created using a track. Both have ids. It is assumed in many nodes that the source track id is the landmark id.
    In this node, we update the landmark id to match its track id. It may have change over operations on tracks.
    """

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
