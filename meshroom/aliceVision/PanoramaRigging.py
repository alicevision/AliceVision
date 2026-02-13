__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class PanoramaRigging(desc.AVCommandLineNode):
    commandLine = "aliceVision_panoramaRigging {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Panorama HDR"
    documentation = """Transform a panorama to a panorama with a rig"""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file with the estimated panorama.",
            value="",
        ),
        desc.File(
            name="rigDescription",
            label="Rig Description",
            description="Input SfMData file containing the rig structure.",
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
