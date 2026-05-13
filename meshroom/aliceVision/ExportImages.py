__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import COLORSPACES, EXR_STORAGE_DATA_TYPE, VERBOSE_LEVEL
from pyalicevision import parallelization as avpar

class ExportImages(desc.AVCommandLineNode):
    """
Export images referenced in the input sfmData by transforming them to adapt to the required target intrinsics.
For example, the target intrinsics may be the same without the distortion.
"""

    commandLine = "aliceVision_exportImages {allParams}"
    size = avpar.DynamicViewsSize("input")
    
    parallelization = desc.Parallelization(blockSize=40)
    commandLineRange = "--rangeStart {rangeStart} --rangeSize {rangeBlockSize}"

    category = "Export"
    inputs = [
        desc.File(
            name="input",
            label="Source SfMData",
            description="Input SfMData file. Contains the original intrinsics of the images.",
            value="",
        ),
        desc.File(
            name="target",
            label="Target SfMData",
            description="This SfMData file contains the required intrinsics for the output images.",
            value="",
        ),
        desc.ListAttribute(
            elementDesc=desc.File(
                name="masksFolder",
                label="Masks Folder",
                description="",
                value="",
            ),
            name="masksFolders",
            label="Masks Folders",
            description="Use masks from specific folder(s). Filename should be the same or the image UID.",
            exposed=True
        ),
        desc.ChoiceParam(
            name="outputFileType",
            label="Output File Type",
            description="Output file type for the exported images.",
            value="exr",
            values=["jpg", "png", "tif", "exr"],
            advanced=True,
        ),
        desc.BoolParam(
            name="evCorrection",
            label="Correct Images Exposure",
            description="Apply a correction on images' exposure value.",
            value=False,
            advanced=True,
        ),
        desc.BoolParam(
            name="exportFullROD",
            label="Export Full ROD",
            description="Export images with the full Region of Definition (ROD). Only supported by the EXR file format.",
            value=False,
            enabled=lambda node: node.outputFileType.value == "exr"
        ),
        desc.ChoiceParam(
            name="namingMode",
            label="Naming Mode",
            description="image naming mode :\n"
                        " - viewid: viewid.ext.\n"
                        " - frameid: Frameid.ext.\n"
                        " - keep: Keep original name.\n",
            value="viewid",
            values=["viewid", "frameid", "keep"],
        ),
        desc.ChoiceParam(
            name="maskExtension",
            label="Mask Extension",
            description="File extension for the masks to use.",
            value="png",
            values=["exr", "jpg", "png"],
        ),
        desc.ChoiceParam(
            name="storageDataType",
            label="Storage Data Type",
            description="Storage image data type:\n"
                        " - float: Use full floating point (32 bits per channel).\n"
                        " - half: Use half float (16 bits per channel).\n"
                        " - halfFinite: Use half float, but clamp values to avoid non-finite values.\n"
                        " - auto: Use half float if all values can fit, else use full float.",
            values=EXR_STORAGE_DATA_TYPE,
            value="halfFinite",
            enabled=lambda node: node.outputFileType.value == "exr"
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
            label="Images Folder",
            description="Output folder.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="undistorted",
            label="Undistorted Images",
            description="List of undistorted images.",
            semantic="image",
            value=lambda attr: getUndistortedPath(attr.node.namingMode.value),
            commandLineGroup="",
            advanced=True,
        ),
         desc.File(
            name="outputSfMData",
            label="Output SfMData",
            description="Path to the target SfMData file updated with transformed images.",
            value="{nodeCacheFolder}/sfm.abc",
        ),
    ]

def getUndistortedPath(namingMode):
    
    replacement = "<FILESTEM>"
    if (namingMode == "viewid"):
        replacement = "<VIEW_ID>"
    elif (namingMode == "frameid"):
        replacement = "<FRAME_ID>"

    return "{nodeCacheFolder}/" + replacement + ".{outputFileTypeValue}"
                    