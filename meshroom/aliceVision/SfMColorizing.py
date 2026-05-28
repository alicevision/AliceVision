__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json

class SfMColorizing(desc.AVCommandLineNode):
    """
Assign RGB colors to the 3D point cloud in an SfMData scene.

Each 3D landmark in the scene is colored by sampling the pixel values from the
input images in which it is visible. When a landmark is seen by multiple cameras,
the colors are averaged to produce a robust estimate. The colored point cloud
can be exported or visualized to assess the quality of the reconstruction.
"""


    commandLine = "aliceVision_sfmColorizing {allParams}"
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
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
    ]
