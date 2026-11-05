__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class AddPoseNoise(desc.AVCommandLineNode):
    '''
Add synthetic noise to the camera poses in an SfMData scene.

This node perturbs each camera's position and orientation by a configurable amount of
Gaussian noise. It is primarily intended for testing and benchmarking purposes, allowing
users to evaluate the robustness of downstream algorithms (e.g., bundle adjustment, depth
map estimation) when the input poses are not perfect.
'''

    commandLine = 'aliceVision_addPoseNoise {allParams}'
    size = desc.DynamicNodeSize('input')

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    category = 'Utils'
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.FloatParam(
            name="positionNoise",
            label="Position Noise",
            description="Noise level to add to view positions.",
            value=0.0,
            range=(0.0, 1.0, 0.025),
        ),
        desc.FloatParam(
            name="rotationNoise",
            label="Rotation Noise",
            description="Noise level to add to view orientations.",
            value=0.0,
            range=(0.0, 1.0, 0.025),
        ),
        desc.IntParam(
            name="randomSeed",
            label="Random Seed",
            description="This seed value will generate a sequence using a linear random generator.\n"
                        "Set -1 to use a random seed.",
            value=5489,
            range=(-1, 10000, 1),
            advanced=True,
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
            value="{nodeCacheFolder}/sfmFiltered.usda",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
