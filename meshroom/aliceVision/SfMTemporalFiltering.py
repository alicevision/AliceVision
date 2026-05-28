__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMTemporalFiltering(desc.AVCommandLineNode):
    '''
This node takes the result of SfM and fine-tune the camera poses so that the camera path is temporally smooth.
'''

    commandLine = 'aliceVision_sfmTempFiltering {allParams}'
    size = desc.DynamicNodeSize('input')

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    category = 'Sparse Reconstruction'
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.BoolParam(
            name="filterPosition",
            label="Filter Positions",
            description="Whether to filter camera positions.",
            value=True,
        ),
        desc.BoolParam(
            name="filterRotation",
            label="Filter Rotations",
            description="Whether to filter camera orientations.",
            value=True,
        ),
        desc.IntParam(
            name="scaleFactor",
            label="Scale Factor",
            description="Scale factor to increase the filter range.",
            value=3,
            range=(1, 20, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="iterationCount",
            label="Iteration Count",
            description="Number of filter iterations.",
            value=100,
            range=(0, 1000, 10),
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
            value="{nodeCacheFolder}/sfmFiltered.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
