__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL
from pyalicevision import parallelization as avpar


class MaskEroding(desc.AVCommandLineNode):
    commandLine = "aliceVision_maskEroding {allParams}"

    size = avpar.DynamicViewsSize("input")
    parallelization = desc.Parallelization(blockSize=50)
    commandLineRange = "--rangeIteration {rangeIteration} --rangeBlocksCount {rangeBlocksCount}"

    category = "Utils"
    documentation = """ Assumes the inputs are binary masks. 
    Erode the valid part of the mask such that a new pixel is valid if and only if the input region is fully valid. """

    inputs = [
        desc.File(
            name="input",
            label="Input Directory",
            description="A directory with a set of mask.",
            value="",
        ),
        desc.IntParam(
            name="radius",
            label="Radius of erosion",
            description="Radius of the erosion filter",
            value=5,
            range=None,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        )
    ]

    outputs = [
        desc.File(
            name="output",
            label="Output",
            description="Path to the output directory.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="masks",
            label="Masks",
            description="Processed masks.",
            semantic="imageList",
            value= "{nodeCacheFolder}/*.exr",
            group="",
        ),
    ]
