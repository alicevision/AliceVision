__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ApplyCalibration(desc.AVCommandLineNode):
    """
Overwrite the intrinsic parameters of cameras in an SfMData scene with a pre-calibrated intrinsic.

This node replaces the intrinsics of all (or selected) cameras in the input SfMData with
the intrinsics from a calibration file. The calibration file can be either an SfMData file
or a dedicated lens calibration file. This is useful when a precise factory or lab calibration
is available and should be enforced instead of the intrinsics estimated during reconstruction.
"""

    commandLine = "aliceVision_applyCalibration {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="calibration",
            label="Calibration",
            description="Calibration file (SfmData or Lens calibration file).",
            value="",
        ),
        desc.BoolParam(
            name="useJson",
            label="Use Lens Calibration File",
            description="Calibration is a Lens calibration file generated using 3Dequalizer instead of an sfmData.",
            value=False,
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
            description="Path to the output SfMData file.",
            value="{nodeCacheFolder}/sfmData.sfm",
        ),
    ]
