__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ListImages(desc.AVCommandLineNode):
    """
Create an SfMData scene from a set of image files or folders.

This node scans the provided image files and directories, reads available metadata
(e.g., EXIF focal length, sensor dimensions, serial number), and produces a minimal
SfMData file listing all discovered views with their associated intrinsic parameters.
The resulting SfMData is suitable as an input for the CameraInit node or other nodes
that accept an SfMData file without requiring reconstructed poses.
"""

    commandLine = "aliceVision_listImages {allParams}"

    category = "Utils"
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
        desc.BoolParam(
            name="isSequence",
            label="Images are a sequence",
            description="The images provided as input are part of a sequence with temporal coherency.",
            value=False,
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
