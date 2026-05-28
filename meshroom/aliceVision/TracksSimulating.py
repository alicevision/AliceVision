__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class TracksSimulating(desc.AVCommandLineNode):
    """
Generate synthetic feature tracks from a known SfMData scene for algorithm testing.

Given an SfMData scene with known camera poses and 3D landmarks, this node simulates
2D feature observations by projecting each landmark into all cameras that can see it.
Configurable Gaussian noise can be added to the 2D observations, and a fraction of them
can be replaced by outliers. The output tracks can be used to test and benchmark
SfM algorithms in a controlled setting with known ground truth.
"""

    commandLine = "aliceVision_tracksSimulating {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
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
        desc.BoolParam(
            name="randomNoiseVariancePerView",
            label="Random Variance Per View",
            description="Use different noise variance per view.",
            value=False,
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
