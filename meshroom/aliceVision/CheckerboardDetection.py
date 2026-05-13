__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL
from pyalicevision import parallelization as avpar

class CheckerboardDetection(desc.AVCommandLineNode):
    """
Detect checkerboard structures in a set of images.
The detection method also supports nested calibration grids.
"""

    commandLine = "aliceVision_checkerboardDetection {allParams}"
    size = avpar.DynamicViewsSize("input")
    
    parallelization = desc.Parallelization(blockSize=5)
    commandLineRange = "--rangeStart {rangeStart} --rangeSize {rangeBlockSize}"

    category = "Other"
    inputs = [
        desc.File(
            name="input",
            label="Input",
            description="Input SfMData file. Viewpoints must correspond to lens calibration grids.",
            value="",
        ),
        desc.BoolParam(
            name="useNestedGrids",
            label="Nested Calibration Grid",
            description="Enable if images contain nested calibration grids. These grids must be centered on the image center.",
            value=False,
        ),
        desc.BoolParam(
            name="doubleSize",
            label="Double Size",
            description="Double the image size prior to processing.",
            value=False,
        ),
        desc.BoolParam(
            name="ignorePixelAspectRatio",
            label="Ignore Pixel Aspect Ratio",
            description="Ignore pixel aspect ratio for detection.",
            value=False,
        ),
        desc.BoolParam(
            name="useAllSeeds",
            label="Use all seeds",
            description="False will ignore seed corner point if it is already part of a detected potential checkerboard.",
            value=False,
        ),
        desc.IntParam(
            name="maxLevels",
            label="Maximum scale for pyramid",
            description="Maximum number of levels used in multiscale point detection.",
            value=2,
            range=(1, 10, 1),
            advanced=True
        ),
        desc.IntParam(
            name="minConsensus",
            label="Minimum merge consensus",
            description="Minimum number of shared corners to merge checkerboards.",
            value=5,
            range=(1, 10, 1),
            advanced=True
        ),
        desc.BoolParam(
            name="exportDebugImages",
            label="Export Debug Images",
            description="Export debug images.",
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
            label="Folder",
            description="Output folder.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="checkerLines",
            enabled=lambda node: node.exportDebugImages.value,
            label="Checker Lines",
            description="Debug images.",
            semantic="image",
            value="{nodeCacheFolder}/<VIEW_ID>.png",
            commandLineGroup="",  # do not export on the command line
        ),
    ]
