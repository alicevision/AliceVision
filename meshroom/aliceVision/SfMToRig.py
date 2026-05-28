__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import os.path

class SfMToRig(desc.AVCommandLineNode):
    """
Convert a set of independently posed cameras in an SfMData into a multi-camera rig.

This node assumes that the input SfMData contains multiple cameras that all observed the
same scene at the same moment in time (e.g., a synchronized multi-camera array). It
groups the cameras into a single rig structure, where each camera is expressed as a
sub-pose relative to the rig's reference frame. The resulting SfMData can then be used
in downstream nodes that explicitly support or require a rig representation.
"""

    commandLine = "aliceVision_sfmToRig {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
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
            description="Path to the output SfM file (in SfMData format).",
            value="{nodeCacheFolder}/sfmData.sfm",
        ),
    ]
