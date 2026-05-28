__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ExportUSD(desc.AVCommandLineNode):
    """
    Convert cameras from an SfM scene into an animated camera in USD format.
    Based on the input image filenames, this node detects video sequences and creates the corresponding animated camera.
    """

    commandLine = "aliceVision_exportUSD {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="SfMData file containing a complete SfM.",
            value="",
        ),
        desc.FloatParam(
            name="frameRate",
            label="Camera Frame Rate",
            description="Define the camera's frames per second.",
            value=24.0,
            range=(1.0, 60.0, 1.0),
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
            label="USD filename",
            description="Output usd filename",
            value="{nodeCacheFolder}/animated.usda",
        )
    ]
