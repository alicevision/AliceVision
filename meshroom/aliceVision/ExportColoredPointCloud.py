__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ExportColoredPointCloud(desc.AVCommandLineNode):
    """
Export the 3D point cloud from an SfMData scene as a colored point cloud.

Each 3D point (landmark) in the scene is assigned a color sampled from the images
in which it is visible. The output is an SfMData file containing the colored point cloud,
which can be further converted to other formats (e.g., PLY) for visualization.
"""

    commandLine = "aliceVision_exportColoredPointCloud {allParams}"

    category = "Export"
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="SfMData file containing a complete SfM.",
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
            label="Point Cloud Filepath",
            description="Output point cloud with visibilities as SfMData file.",
            value="{nodeCacheFolder}/pointCloud.abc",
        ),
    ]
