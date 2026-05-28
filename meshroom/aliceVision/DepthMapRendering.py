__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class DepthMapRendering(desc.AVCommandLineNode):
    """
Render synthetic depth maps for each calibrated camera using a reference 3D mesh.

Given an SfMData file with known camera poses and intrinsics, and a 3D mesh, this node
rasterizes the mesh from each camera viewpoint to produce per-view depth maps. These
rendered depth maps can be used as ground-truth references for evaluating depth estimation
quality, or as initialization for depth map fusion.
"""

    commandLine = "aliceVision_depthMapRendering {allParams}"

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="mesh",
            label="Input Mesh",
            description="Input mesh file.",
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
            label="Folder",
            description="Output folder.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="depth",
            label="Depth Maps",
            description="Rendered depth maps.",
            semantic="image",
            value="{nodeCacheFolder}/<VIEW_ID>_depthMap.exr",
            commandLineGroup="",  # do not export on the command line
        ),
        desc.File(
            name="mask",
            label="Masks",
            description="Masks.",
            semantic="image",
            value="{nodeCacheFolder}/<VIEW_ID>_mask.exr",
            commandLineGroup="",  # do not export on the command line
        ),
    ]
