__version__ = '6.1'

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class DistortionCalibration(desc.AVCommandLineNode):
    """Calibration of a camera/lens couple distortion using a full screen checkerboard."""

    commandLine = "aliceVision_distortionCalibration {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Other"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.File(
            name="checkerboards",
            label="Checkerboards Folder",
            description="Folder containing checkerboard JSON files.",
            value="",
        ),
        desc.ChoiceParam(
            name="undistortionModelName",
            label="Undistortion Model",
            description="model used to estimate undistortion.",
            value="3deanamorphic4",
            values=["3deanamorphic4", "3declassicld", "3deradial4"],
        ),
        desc.BoolParam(
            name="bestOnly",
            label="Keep best image",
            description="All detected checkerboards across images sharing the same intrinsic are used for distortion calibration. Once this option is enabled, only the best image is selected to optimize the intrinsics rather than utilizing all available images.",
            value=False,
        ),
        desc.BoolParam(
            name="handleSqueeze",
            label="Handle Squeeze",
            description="Estimate squeeze.",
            value=True,
        ),
        desc.BoolParam(
            name="isDesqueezed",
            label="Is Desqueezed",
            description="True if the input image is already desqueezed.",
            value=False,
        ),
        desc.FloatParam(
            name="forcedPixelAspectRatio",
            label="Force PixelAspect Ratio",
            description="Force pixel aspect ratio value, overriding metadatas. Ignored if less than or equal 0.0.",
            value=0.0,
            range=(0.0, 2.0, 0.1),
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
            label="SfMData File",
            description="Path to the output SfMData file.",
            value="{nodeCacheFolder}/sfmData.sfm",
        ),
    ]
