__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class NormalMapRendering(desc.AVCommandLineNode):
    """
Render synthetic normal maps for each calibrated camera using a reference 3D mesh.

Given an SfMData file with known camera poses and intrinsics, and a 3D mesh, this node
rasterizes the mesh from each camera viewpoint and outputs per-view normal maps in camera
space. The rendered normal maps can serve as ground truth for evaluating photometric stereo
algorithms, or as input to normal integration for surface refinement.
"""

    commandLine = "aliceVision_normalMapRendering {allParams}"

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
            name="normal",
            label="Normal Maps",
            description="Rendered normal maps.",
            semantic="image",
            value="{nodeCacheFolder}/<VIEW_ID>_normalMap.exr",
            commandLineGroup="",  # do not export on the command line
        ),
    ]
