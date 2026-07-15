__version__ = "2.1"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class NodalSfM(desc.AVCommandLineNode):
    """
Perform Structure-from-Motion for scenes captured with a camera undergoing pure rotation.

Unlike standard SfM pipelines that require translation between views to triangulate 3D
points, this node is designed for panoramic capture setups where the camera rotates around
a single nodal point (no parallax). It estimates the relative rotation between each pair
of images and recovers the global camera orientations. The output is an SfMData with
calibrated rotations but no translational component, suitable for panorama stitching.
"""

    commandLine = "aliceVision_nodalSfM {allParams}"
    size = desc.DynamicNodeSize("input")
    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE
    
    category = "Sparse Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Input tracks file.",
            value="",
        ),
        desc.File(
            name="pairs",
            label="Pairs File",
            description="Information on pairs.",
            value="",
        ),
        desc.FloatParam(
            name="maxReprojectionError",
            label="Max Reprojection Error",
            description="Maximum reprojection error (pixels) to accept a track as a landmark observation.",
            value=4.0,
            range=(0.1, 20.0, 0.1),
        ),
        desc.IntParam(
            name="minInliers",
            label="Min Inliers",
            description="Minimum number of AC-RANSAC inliers required to localize a view.",
            value=35,
            range=(5, 1000, 1),
        ),
        desc.IntParam(
            name="maxRansacIterations",
            label="Max RANSAC Iterations",
            description="Maximum number of AC-RANSAC iterations for rotation estimation.",
            value=1024,
            range=(100, 10000, 100),
        ),
        desc.FloatParam(
            name="outlierThreshold",
            label="Outlier Threshold",
            description="Pixel residual threshold for outlier removal after bundle adjustment.",
            value=2.0,
            range=(0.1, 20.0, 0.1),
        ),
        desc.IntParam(
            name="minObservations",
            label="Min Observations",
            description="Minimum number of observations required to keep a landmark after outlier removal.",
            value=2,
            range=(1, 10, 1),
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
            value="{nodeCacheFolder}/sfm.usda",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]
