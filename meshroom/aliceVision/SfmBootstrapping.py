__version__ = "4.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMBootStrapping(desc.AVCommandLineNode):
    commandLine = 'aliceVision_sfmBootstrapping {allParams}'
    size = desc.DynamicNodeSize('input')

    category = 'Sparse Reconstruction'
    documentation = '''
'''

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.File(
            name="meshFilename",
            label="Mesh File",
            description="Mesh file (*.obj).",
            value="",
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
            value="",
        ),
        desc.FloatParam(
            name="minAngleInitialPair",
            label="Min Angle Initial Pair",
            description="Minimum angle for the initial pair.",
            value=5.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="maxAngleInitialPair",
            label="Max Angle Initial Pair",
            description="Maximum angle for the initial pair.",
            value=40.0,
            range=(0.1, 60.0, 0.1),
            advanced=True,
        ),
        desc.File(
            name="initialPairA",
            label="Initial Pair A",
            description="View ID of the first image.",
            value="",
        ),
        desc.File(
            name="initialPairB",
            label="Initial Pair B",
            description="View ID of the second image.",
            value="",
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
            value="{nodeCacheFolder}/bootstrap.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
