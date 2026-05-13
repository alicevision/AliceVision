__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

class NormalIntegration(desc.CommandLineNode):
    """
Reconstruct a depth map by integrating a surface normal map.

Given a per-pixel normal map (as produced by the PhotometricStereo pipeline), this node
integrates the normals to recover the underlying surface depth. Normal integration solves
a Poisson-like equation to find the depth field whose gradients best match the input normals.
The result is a depth map that can be used for high-frequency surface detail recovery.

Note: This node is currently under active development and may produce incomplete results.
"""

    commandLine = "aliceVision_normalIntegration {allParams}"
    category = "Photometric Stereo"
    inputs = [
        desc.File(
            name="inputPath",
            label="Normal Maps Folder",
            description="Path to the folder containing the normal maps and the masks.",
            value="",
         ),
        desc.File(
            name="sfmDataFile",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.IntParam(
            name="downscale",
            label="Downscale Factor",
            description="Downscale factor for faster results.",
            value=1,
            range=(1, 10, 1),
            advanced=True,
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
            name="depthMap",
            label="Depth Map Camera",
            description="Generated depth in the camera coordinate system.",
            semantic="image",
            value="{nodeCacheFolder}/<POSE_ID>_depthMap.exr",
            commandLineGroup="", # do not export on the command line
        )
    ]
