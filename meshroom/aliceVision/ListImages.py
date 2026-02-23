__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ListImages(desc.AVCommandLineNode):
    commandLine = "aliceVision_listImages {allParams}"

    category = "Utils"
    documentation = """Generate a sfmData using a set of images."""

    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="inputFile",
                label="image(s) file or folder",
                description="Path to an image file or a folder.",
                value="",
            ),
            name="input",
            label="Input Files",
            description="Set of paths to image files and/or folders.",
            exposed=True,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        )
    ]

    outputs = [
        desc.File(
            name="output",
            label="Output",
            description="Path to the generated SfmData.",
            value="{nodeCacheFolder}/sfmData.sfm",
        ),
    ]
