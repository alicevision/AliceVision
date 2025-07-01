__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class TracksSimulating(desc.AVCommandLineNode):
    commandLine = 'aliceVision_tracksSimulating {allParams}'
    size = desc.DynamicNodeSize('input')

    category = 'Utils'
    documentation = '''
    Generate tracks from an SfmData input.
    '''

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
            exposed=True,
        ),
        desc.FloatParam(
            name="sigmaNoise",
            label="Additional Noise",
            description="Observation coordinates are modified with an additive Gaussian noise. The value is the variance of the Gaussian (in pixels).",
            value=0.0,
            invalidate=True,
            advanced=True,
        ),
        desc.FloatParam(
            name="outlierRatio",
            label="Outlier Ratio",
            description="The ratio of outliers with respect to the observations count.",
            value=0.0,
            invalidate=True,
            advanced=True,
        ),
        desc.FloatParam(
            name="outlierEpipolarRatio",
            label="Outlier With Epipolar Constraint",
            description="Proportion of outliers which are still respecting the epipolar constraint.",
            value=0.0,
            invalidate=True,
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
            label="Tracks",
            description="Path to the output tracks file.",
            value="{nodeCacheFolder}/tracksFile.json",
        ),
    ]
