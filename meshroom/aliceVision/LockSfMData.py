__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL

import os.path


class LockSfMData(desc.AVCommandLineNode):
    commandLine = "aliceVision_lockSfmData {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    documentation = """
Lock specific elements of an SfMData scene so that they are kept fixed during subsequent
bundle adjustment steps.

The following elements can be locked independently:
 * **Camera Intrinsics**: focal length, principal point, and distortion parameters.
 * **Camera Poses**: position and orientation of all reconstructed cameras.
 * **Landmarks**: 3D points of the sparse point cloud, optionally filtered by describer type.
"""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.BoolParam(
            name="lockIntrinsics",
            label="Lock Intrinsics",
            description="Lock all camera intrinsics (focal length, principal point, distortion).",
            value=False,
        ),
        desc.BoolParam(
            name="lockPoses",
            label="Lock Poses",
            description="Lock all camera poses (position and orientation).",
            value=False,
        ),
        desc.BoolParam(
            name="lockLandmarks",
            label="Lock Landmarks",
            description="Lock 3D landmarks (sparse point cloud).",
            value=False,
        ),
        desc.ChoiceParam(
            name="lockLandmarkTypes",
            label="Landmark Types To Lock",
            description="Describer types of landmarks to lock.\n"
                        "If empty, all landmark types will be locked.",
            values=DESCRIBER_TYPES,
            value=[],
            exclusive=False,
            joinChar=",",
            enabled=lambda node: node.lockLandmarks.value,
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
            description="Output SfMData file with locked elements.",
            value=lambda attr: "{nodeCacheFolder}/" + (os.path.splitext(os.path.basename(attr.node.input.value))[0] or "sfmData") + ".abc",
        ),
    ]
