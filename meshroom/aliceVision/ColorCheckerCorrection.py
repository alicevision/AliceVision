__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import EXR_STORAGE_DATA_TYPE, VERBOSE_LEVEL

import os.path

def outputImagesValueFunct(attr):
    basename = os.path.basename(attr.node.input.value)
    fileStem = os.path.splitext(basename)[0]
    inputExt = os.path.splitext(basename)[1]
    outputExt = ('.' + attr.node.extension.value) if attr.node.extension.value else None

    if inputExt in ['.abc', '.sfm']:
        fileStem = '<FILESTEM>' if attr.node.keepImageName.value else '<VIEW_ID>'
        # If we have an SfM in input
        return "{nodeCacheFolder}/" + fileStem + (outputExt or '.*')

    if inputExt:
        # If we have one or multiple files in input
        return "{nodeCacheFolder}/" + fileStem + (outputExt or inputExt)

    if '*' in fileStem:
        # The fileStem of the input param is a regular expression,
        # so even if there is no file extension,
        # we consider that the expression represents files.
        return "{nodeCacheFolder}/" + fileStem + (outputExt or '.*')

    # No extension and no expression means that the input param is a folder path
    return "{nodeCacheFolder}/" + '*' + (outputExt or '.*')

class ColorCheckerCorrection(desc.AVCommandLineNode):
    """
(BETA) \\
Performs color calibration from Macbeth color checker chart.

The node assumes all the images to process are sharing the same colorimetric properties.
All the input images will get the same correction.

If multiple color charts are submitted, only the first one will be taken in account.
"""

    commandLine = "aliceVision_colorCheckerCorrection {allParams}"
    size = desc.DynamicNodeSize("input")
    # parallelization = desc.Parallelization(blockSize=40)
    # commandLineRange = '--rangeStart {rangeStart} --rangeSize {rangeBlockSize}'

    inputs = [
        desc.File(
            name="inputData",
            label="Color Checker Data",
            description="Position and colorimetric data of the color checker.",
            value="",
        ),
        desc.File(
            name="input",
            label="Input",
            description="Input SfMData file, image filenames or regex(es) on the image file path.\n"
                        "Supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more.",
            value="",
        ),
        desc.ChoiceParam(
            name="correctionMethod",
            label="Correction Level",
            description="Level of correction:\n"
                        " - luminance: Adjust luminance level only.\n"
                        " - whiteBalance: Apply white balancing in addition to luminance adjustment.\n"
                        " - full: Full color correction."
                        " - bypass: Do nothing.",
            value="luminance",
            values=["luminance", "whiteBalance", "full", "bypass"],
            exclusive=True,
        ),
        desc.BoolParam(
            name="useBestColorCheckerOnly",
            label="Use Best Color Chart Only",
            description="If checked, use only the color chart with the best orientation and size to compute the color correction model.\n"
                        "If unchecked, combine all detected color checkers.",
            value=True,
        ),
        desc.BoolParam(
            name="keepImageName",
            label="Keep Image Name",
            description="Keep image names if different from the view Ids.",
            value=True,
        ),
        desc.ChoiceParam(
            name="extension",
            label="Output File Extension",
            description="Output image file extension.",
            value="exr",
            values=["exr", ""],
            exclusive=True,
        ),
        desc.ChoiceParam(
            name="storageDataType",
            label="EXR Storage Data Type",
            description="Storage data type for EXR output:\n"
                        " - float: Use full floating point (32 bits per channel).\n"
                        " - half: Use half float (16 bits per channel).\n"
                        " - halfFinite: Use half float, but clamp values to avoid non-finite values.\n"
                        " - auto: Use half float if all values can fit, else use full float.",
            values=EXR_STORAGE_DATA_TYPE,
            value="float",
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
            name="outSfMData",
            label="SfMData",
            description="Output SfMData.",
            value=lambda attr: ("{nodeCacheFolder}/" + os.path.basename(attr.node.input.value)) if (os.path.splitext(attr.node.input.value)[1] in [".abc", ".sfm"]) else "",
            commandLineGroup="",  # do not export on the command line
        ),
        desc.File(
            name="output",
            label="Folder",
            description="Output images folder.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="outputImages",
            label="Images",
            description="Output images.",
            semantic="image",
            value=outputImagesValueFunct,
            commandLineGroup="",  # do not export on the command line
        ),
    ]
