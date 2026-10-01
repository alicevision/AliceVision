__version__ = "1.2"

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

An optional **Selected Views** SfMData can be provided to restrict locking to elements
associated with that subset of views.

The following elements can be locked independently:
 * **Camera Intrinsics**: all intrinsic parameters, or specific sub-parts:
   * **Focal Length**: scale parameters of the camera model.
   * **Principal Point**: offset parameters of the camera model.
   * **Distortion**: distortion parameters of the camera model.
 * **Camera Poses**: position and orientation of all reconstructed cameras.
 * **Landmarks**: 3D points of the sparse point cloud, optionally filtered by describer type
   and/or by landmark selection mode (fully or partially contained within the selected views).
"""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="selectedViews",
            label="Selected Views",
            description="Optional SfMData file used to define a subset of views.\n"
                        "When provided, locking is restricted to elements associated with those views.",
            value="",
        ),
        desc.BoolParam(
            name="lockIntrinsics",
            label="Lock Intrinsics",
            description="Lock camera intrinsics.",
            value=False,
        ),
        desc.BoolParam(
            name="lockFocalLength",
            label="Lock Focal Length",
            description="Lock the focal length of camera intrinsics.",
            value=True,
            enabled=lambda node: node.lockIntrinsics.value,
        ),
        desc.BoolParam(
            name="lockPrincipalPoint",
            label="Lock Principal Point",
            description="Lock the principal point of camera intrinsics.",
            value=True,
            enabled=lambda node: node.lockIntrinsics.value,
        ),
        desc.BoolParam(
            name="lockDistortion",
            label="Lock Distortion",
            description="Lock the distortion parameters of camera intrinsics.",
            value=True,
            enabled=lambda node: node.lockIntrinsics.value,
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
            name="landmarkSelectionMode",
            label="Landmark Selection Mode",
            description="Determines which landmarks to lock when Selected Views is provided:\n"
                        " - fully_contained: lock only landmarks whose all observations belong to the selected views.\n"
                        " - partially_contained: lock landmarks with at least one observation in the selected views.",
            value="fully_contained",
            values=["fully_contained", "partially_contained"],
            exclusive=True,
            enabled=lambda node: node.lockLandmarks.value and bool(node.selectedViews.value),
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
