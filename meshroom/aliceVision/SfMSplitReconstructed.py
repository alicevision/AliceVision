__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMSplitReconstructed(desc.AVCommandLineNode):
    """
Split an SfMData scene into two separate files based on reconstruction status.

This node separates the views in the input SfMData into two groups:
 - **Reconstructed**: Views for which a camera pose has been successfully estimated.
 - **Non-reconstructed**: Views that could not be localized during SfM.

The two resulting SfMData files can then be processed independently, for example to
focus further processing on only the reconstructed cameras or to attempt localizing
the remaining views using a different strategy.
"""

    commandLine = "aliceVision_sfmSplitReconstructed {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
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
            name="reconstructedOutput",
            label="Reconstructed SfMData File",
            description="SfMData file containing the reconstructed cameras.",
            value="{nodeCacheFolder}/sfmReconstructed.abc",
        ),
        desc.File(
            name="notReconstructedOutput",
            label="Not Reconstructed SfMData File",
            description="SfMData file containing the non-reconstructed cameras.",
            value="{nodeCacheFolder}/sfmNonReconstructed.abc",
        ),
    ]
