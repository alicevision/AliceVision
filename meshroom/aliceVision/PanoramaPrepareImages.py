__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import os.path


class PanoramaPrepareImages(desc.AVCommandLineNode):
    """
Prepare images for the Panorama pipeline by ensuring that all image orientations are consistent.

EXIF orientation tags or inconsistent camera orientations can cause issues in panorama
stitching. This node reads the orientation metadata from each view and applies the necessary
rotations so that all images are stored with a canonical upright orientation. It also
updates the SfMData accordingly to ensure that subsequent nodes process images correctly.
"""

    commandLine = "aliceVision_panoramaPrepareImages {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Panorama HDR"
    inputs = [
        desc.File(
            name="input",
            label="Input",
            description="SfMData file.",
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
            description="Output SfMData file.",
            value=lambda attr: "{nodeCacheFolder}/" + os.path.basename(attr.node.input.value),
        ),
    ]
