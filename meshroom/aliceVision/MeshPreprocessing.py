__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class MeshPreprocessing(desc.AVCommandLineNode):
    """
"""

    commandLine = "aliceVision_meshPreprocessing {allParams}"

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Mesh file",
            description="Input mesh file",
            value="",
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
            label="Output mesh binary File",
            description="Path to the output file",
            value="{nodeCacheFolder}/mesh.bin",
        )
    ]
    