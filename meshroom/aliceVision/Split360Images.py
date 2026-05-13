__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class Split360InputNodeSize(desc.DynamicNodeSize):
    """
    The Split360Images will increase the amount of views in the SfMData.
    This class converts the number of input views into the number of split output views.
    """
    def __call__(self, node):
        s = super(Split360InputNodeSize, self).__call__(node)
        factor = 0
        mode = node.attribute('splitMode')
        if mode.value == 'equirectangular':
            factor = node.attribute('equirectangularGroup.equirectangularNbSplits').value
        elif mode.value == 'dualfisheye':
            factor = 2
        return s * factor


class Split360Images(desc.AVCommandLineNode):
    """
Extract multiple perspective images from a single equirectangular or dual-fisheye image.

360-degree images captured as equirectangular projections or dual-fisheye pairs cannot
be directly used by standard SfM pipelines. This node decomposes each input image into
a configurable number of overlapping perspective crops, creating a larger set of
conventional perspective views that can be processed by the regular reconstruction pipeline.

Two split modes are supported:
 - **equirectangular**: Splits an equirectangular panoramic image into multiple
   perspective crops distributed around the sphere.
 - **dualfisheye**: Converts a dual-fisheye image into two separate fisheye views.
"""

    commandLine = "aliceVision_split360Images {allParams}"
    size = Split360InputNodeSize("input")
    
    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input",
            description="Single image, image folder or SfMData file.",
            value="",
        ),
        desc.ChoiceParam(
            name="splitMode",
            label="Split Mode",
            description="Split mode (equirectangular, dualfisheye).",
            value="equirectangular",
            values=["equirectangular", "dualfisheye"],
        ),
        desc.GroupAttribute(
            name="dualFisheyeGroup",
            label="Dual Fisheye",
            description="Dual Fisheye.",
            commandLineGroup=None,
            enabled=lambda node: node.splitMode.value == "dualfisheye",
            items=[
                desc.ChoiceParam(
                    name="dualFisheyeOffsetPresetX",
                    label="X Offset Preset",
                    description="Dual-Fisheye X offset preset.",
                    value="center",
                    values=["center", "left", "right"],
                ),
                desc.ChoiceParam(
                    name="dualFisheyeOffsetPresetY",
                    label="Y Offset Preset",
                    description="Dual-Fisheye Y offset preset.",
                    value="center",
                    values=["center", "top", "bottom"],
                ),
                desc.ChoiceParam(
                    name="dualFisheyeCameraModel",
                    label="Camera Model",
                    description="Dual-Fisheye camera model.",
                    value="fisheye4",
                    values=["fisheye4", "equidistant_r3"],
                ),
            ],
        ),
        desc.GroupAttribute(
            name="equirectangularGroup",
            label="Equirectangular",
            description="Equirectangular",
            commandLineGroup=None,
            enabled=lambda node: node.splitMode.value == "equirectangular",
            items=[
                desc.IntParam(
                    name="equirectangularNbSplits",
                    label="Nb Splits",
                    description="Equirectangular number of splits.",
                    value=2,
                    range=(1, 100, 1),
                ),
                desc.IntParam(
                    name="equirectangularSplitResolution",
                    label="Split Resolution",
                    description="Equirectangular split resolution.",
                    value=1200,
                    range=(100, 10000, 1),
                ),
                desc.BoolParam(
                    name="equirectangularPreviewMode",
                    label="Preview Mode",
                    description="Export a SVG file that simulates the split.",
                    value=False,
                ),
                desc.FloatParam(
                    name="fov",
                    label="Field Of View",
                    description="Field of View to extract (in degrees).",
                    value=110.0,
                    range=(0.0, 180.0, 1.0),
                ),
            ],
        ),
        desc.ChoiceParam(
            name="extension",
            label="Output File Extension",
            description="Output image file extension.",
            value="",
            values=["", "exr", "jpg", "tiff", "png"],
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
            label="Folder",
            description="Output folder for extracted frames.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="outSfMData",
            label="SfMData File",
            description="Output SfMData file.",
            value="{nodeCacheFolder}/rig.sfm",
        ),
    ]
