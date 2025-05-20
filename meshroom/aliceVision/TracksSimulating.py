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
            label="Additional noise",
            description="Observation coordinates are modified with an additive gaussian noise. The value is the variance of the gaussian (in pixels).",
            value=0.0,
            invalidate=True,
            advanced=True,
        ),
        desc.FloatParam(
            name="outlierRatio",
            label="Outlier ratio",
            description="What is the ratio of outliers wrt the observations count ?",
            value=0.0,
            invalidate=True,
            advanced=True,
        ),
        desc.FloatParam(
            name="outlierEpipolarRatio",
            label="Outlier with epipolar constraint",
            description="What is the proportion of outliers which are still respecting the epipolar constraint ?",
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
