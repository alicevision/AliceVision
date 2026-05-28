__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMDataIntersection(desc.AVCommandLineNode):
    commandLine = "aliceVision_sfmDataIntersection {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    documentation = """
Filters the 3D landmarks of a SfMData file to keep only the ones that have
observations from both camera group A and camera group B.

The main input SfMData provides all the content (views, poses, intrinsics, landmarks).
InputA and InputB each define a subset of cameras (by their view IDs) that must both
be represented in the observations of a landmark for it to be retained.

This is typically used after a merge (e.g. SfMMerge) to extract only the landmarks
that are co-observed by cameras from two distinct acquisition groups.
"""

    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Main SfMData file providing all views, poses, intrinsics and landmarks.",
            value="",
        ),
        desc.File(
            name="inputA",
            label="Input SfMData A",
            description="SfMData file whose view IDs define camera group A.",
            value="",
        ),
        desc.File(
            name="inputB",
            label="Input SfMData B",
            description="SfMData file whose view IDs define camera group B.",
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
            description="Path to the output SfMData file containing only intersection landmarks.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
    ]
