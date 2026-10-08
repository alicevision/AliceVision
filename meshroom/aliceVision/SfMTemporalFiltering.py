__version__ = "1.1"

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
            name="iterationCount",
            label="Iteration Count",
            description="Number of filter iterations.",
            value=100,
            range=(0, 1000, 10),
            advanced=True,
        ),
        desc.IntParam(
            name="scaleFactor",
            label="Scale Factor",
            description="Scale factor to increase the filter range.",
            value=3,
            range=(1, 20, 1),
            advanced=True,
        ),
        desc.BoolParam(
            name="limitReprojError",
            label="Limit Reprojection Error",
            description="Reduces the number of filter iterations to limit reprojection error.",
            value=False,
            advanced=True,
            commandLineGroup=None,
        ),
        desc.FloatParam(
            name="maxErrorIncreasePos",
            label="Position Reproj Error Ratio",
            description="The maximum reprojection error increase ratio for the camera positions.",
            value=.1,
            range=(0.0, 1.0, 0.02),
            advanced=True,
            enabled=lambda node: node.limitReprojError.value,
        ),
        desc.FloatParam(
            name="maxErrorIncreaseRot",
            label="Rotation Reproj Error Ratio",
            description="The maximum reprojection error increase ratio for the camera orientations.",
            value=.1,
            range=(0.0, 1.0, 0.02),
            advanced=True,
            enabled=lambda node: node.limitReprojError.value,
        ),
        desc.IntParam(
            name="minIterationCount",
            label="Minimum Iteration Count",
            description="The minimum number of filter iterations to apply at the smallest scales.",
            value=50,
            range=(0, 1000, 10),
            advanced=True,
            enabled=lambda node: node.limitReprojError.value,
        ),
        desc.IntParam(
            name="minScaleFactor",
            label="Minimum Scale Factor",
            description="The minimum scale to apply the filter with the minimum iteration count.",
            value=3,
            range=(1, 20, 1),
            advanced=True,
            enabled=lambda node: node.limitReprojError.value,
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
