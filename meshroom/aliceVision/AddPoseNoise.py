__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class AddPoseNoise(desc.AVCommandLineNode):
    commandLine = 'aliceVision_addPoseNoise {allParams}'
    size = desc.DynamicNodeSize('input')

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    category = 'Utils'
    documentation = '''
This node adds noise to view positions and view orientations
'''

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
            value="{nodeCacheFolder}/sfmFiltered.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
