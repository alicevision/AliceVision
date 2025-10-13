__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class TracksShapesInjecting(desc.AVCommandLineNode):
    commandLine = "aliceVision_tracksShapesInjecting {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    documentation = """
This node creates tracks from keyable points drawn inside meshroom
"""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
            exposed=True,
        ),
        desc.File(
            name="shapesFile",
            label="Shapes File",
            description="Shapes File used to load shapes from meshroom.",
            value="",
            exposed=True,
        ),
        desc.BoolParam(
            name="markAsSpecial",
            label="Is survey",
            description="Consider those tracks as 'special' : non removable and very precise.",
            value=True,
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
            label="Tracks",
            description="Path to the output tracks file.",
            value="{nodeCacheFolder}/tracksFile.json",
        ),
    ]
