__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SampleScene(desc.AVCommandLineNode):
    """
Generate a synthetic SfMData scene for testing and development purposes.

This node creates a simple artificial scene (a cube or a sphere) populated with
virtual cameras placed around the object and 3D landmarks sampled from the surface.
The resulting SfMData file contains ground-truth camera poses and 3D point positions
and can be used to validate SfM algorithms or to generate synthetic feature tracks
with TracksSimulating.
"""

    commandLine = "aliceVision_generateSampleScene {allParams}"

    category = "Utils"
    inputs = [
        desc.ChoiceParam(
            name="scene",
            label="Sample Scene",
            description="Type of sample scene to generate (cube or sphere).",
            value="sphere",
            values=["cube", "sphere"],
        ),
        desc.IntParam(
            name="randomSeed",
            label="Random Seed",
            description="This seed value will generate a sequence using a linear random generator.\n"
                        "Set -1 to use a random seed.",
            value=1,
            range=(-1, 10000, 1),
            advanced=True,
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
