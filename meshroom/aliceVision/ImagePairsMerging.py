__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ImagePairsMerging(desc.AVCommandLineNode):
    commandLine = "aliceVision_imagePairsMerging {allParams}"

    category = "Sparse Reconstruction"
    documentation = """Merge multiple image pairs files into a single one."""

    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input Image Pairs File",
                description="An image pairs file.",
                value="",
            ),
            name="inputs",
            label="Inputs",
            description="Set of image pairs files (at least 1 is required).",
            exposed=True,
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
            label="Image Pairs",
            description="Filepath to the output file with the merged list of image pairs.",
            value="{nodeCacheFolder}/imagePairs.txt",
        ),
    ]
