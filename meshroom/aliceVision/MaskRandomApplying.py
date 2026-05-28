__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL



class MaskRandomApplying(desc.AVCommandLineNode):
    """
Replace masked-out pixels in images with random noise.

For each image, pixels whose corresponding mask value is zero (i.e., the background) are
overwritten with random values. This can be used as a data augmentation technique or to
prevent deep learning models from learning spurious correlations from background content,
ensuring that only the foreground object influences the network predictions.
"""

    commandLine = "aliceVision_maskRandomApplying {allParams}"

    size = desc.DynamicNodeSize("input")
    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="SfmData",
            description="Input SfmData with the list of views to process",
            value="",
        ),
        desc.File(
            name="masks",
            label="Input Masks",
            description="Input Masks assumed to have same names than input images",
            value="",
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
            name="outputSfmData",
            label="Output",
            description="Path to the output sfmData.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
        desc.File(
            name="outputDirectory",
            label="Output Images Directory",
            description="Path to the directory with modified images.",
            value="{nodeCacheFolder}",
        )
    ]
