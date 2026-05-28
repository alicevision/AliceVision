__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class PanoramaRigging(desc.AVCommandLineNode):
    """
Convert a panorama captured with a single moving camera into a rig-based representation.

When a panorama is captured by rotating a single camera around its nodal point, each
image is treated as an independent view. This node re-expresses the same panorama as
a virtual multi-camera rig, where all cameras share a common rig pose. This rig-based
representation is required for certain downstream processing steps that expect a rigid
multi-camera configuration.
"""

    commandLine = "aliceVision_panoramaRigging {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Panorama HDR"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file with the estimated panorama.",
            value="",
        ),
        desc.File(
            name="rigDescription",
            label="Rig Description",
            description="Input SfMData file containing the rig structure.",
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
            label="SfM File",
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/panorama.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        ),
    ]
