__version__ = "3.1"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL
from pyalicevision import parallelization as avpar

class RelativePoseEstimating(desc.AVCommandLineNode):
    """
Estimate the relative pose between each pair of views that share feature track observations.

For every image pair with sufficient matching features, this node estimates the relative
rotation and translation using robust geometric methods (e.g., RANSAC with the essential
or fundamental matrix). The resulting relative poses are used by subsequent global or
incremental SfM nodes to recover the absolute camera positions and orientations.
Pure rotation pairs (with no translation baseline) can be explicitly handled as well.
"""

    commandLine = "aliceVision_relativePoseEstimating {allParams}"
    size = avpar.DynamicViewsSize("input")
    
    parallelization = desc.Parallelization(blockSize=25)
    commandLineRange = "--rangeIteration {rangeIteration} --rangeBlocksCount {rangeBlocksCount}"

    category = "Sparse Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.BoolParam(
            name="enforcePureRotation",
            label="Enforce pure rotation",
            description="Enforce pure rotation as a model",
            value=False,
        ),
        desc.IntParam(
            name="countIterations",
            label="Ransac Max Iterations",
            description="Maximal number of iterations.",
            value=1024,
            range=(1024, 500000, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="minInliers",
            label="Ransac Min Inliers",
            description="Minimal allowed inliers in two view relationship.",
            value=35,
            range=(1, 1000, 1),
            advanced=True,
        ),
        desc.FloatParam(
            name="distanceThreshold",
            label="Distance Threshold",
            description="Threshold on geometric distance (epipolar distance or reprojection distance for pure rotation)",
            value=4.0,
            range=(0.0, 50.0, 1.0),
            advanced=True,
        ),
        desc.File(
            name="imagePairsList",
            label="Image Pairs",
            description="Path to a file which contains the list of image pairs to match.",
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
            name="output",
            label="Pairs Info",
            description="Path to the output Pairs info files directory.",
            value="{nodeCacheFolder}",
        ),
    ]
