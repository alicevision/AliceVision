__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SampleScene(desc.AVCommandLineNode):
    commandLine = "aliceVision_generateSampleScene {allParams}"

    category = "Utils"
    documentation = """
Generate a sample scene and a valid SfMData with camera poses, and save it to a given file path.
"""

    inputs = [
        desc.ChoiceParam(
            name="scene",
            label="Sample Scene",
            description="Type of sample scene to generate (cube or sphere).",
            value="sphere",
            values=["cube", "sphere"],
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="SfMData",
            description="SfMData file to generate.",
            value="{nodeCacheFolder}/sampleScene.sfm",
        ),
    ]
