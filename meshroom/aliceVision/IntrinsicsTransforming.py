__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

class IntrinsicsTransforming(desc.AVCommandLineNode):
    """
Convert all camera intrinsic models in an SfMData scene to a different camera type.

Different pipeline stages may require or produce cameras with specific lens models
(e.g., pinhole, radial distortion, fisheye). This node re-fits all intrinsics in
the input SfMData to the chosen camera type, using the track observations to optimize
the new model parameters. The output SfMData contains cameras of the target type
while preserving the reconstructed poses and 3D structure.
"""

    commandLine = "aliceVision_intrinsicsTransforming {allParams}"
    size = desc.DynamicNodeSize("input")
    
    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="inputTracks",
            label="Input Tracks",
            description="Input Tracks file.",
            value="",
        ),
        desc.ChoiceParam(
            name="type",
            label="Camera Type",
            description="Mathematical model used to represent a camera:\n"
                        " - pinhole: Simplest projective camera model without optical distortion "
                        "(focal and optical center).\n"
                        " - equirectangular: Projection model used in panoramas.\n",
            value="pinhole",
            values=["pinhole", "equidistant", "equirectangular"],
        ),
        desc.FloatParam(
            name="fakeFov",
            label="Virtual FOV",
            description="If the input intrinsic is not a pinhole but the output is, what is the virtual FOV requested.",
            value=90.0,
            range=(1.0, 179.0, 0.1),
        ),
        desc.FloatParam(
            name="scaleFactor",
            label="Scale Factor",
            description="Rescale the size of the images in the sfmData description",
            value=1.0,
            range=(0.0, 1.0, 0.1),
            enabled=lambda node: node.type.value == "pinhole"
        ),
        desc.BoolParam(
            name="correctPrincipalPoint",
            label="Correct Principal Point",
            description="Force principal point to image center.",
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
            label="Output SfMData",
            description="Output SfMData file.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
        desc.File(
            name="outputTracks",
            label="Output Tracks",
            description="Output Tracks file.",
            value="{nodeCacheFolder}/tracksFile.json",
        ),
    ]
