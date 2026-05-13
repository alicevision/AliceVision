__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

class LidarDecimating(desc.AVCommandLineNode):
    """
Decimate and simplify meshes reconstructed from LiDAR point cloud data.

LiDAR-based reconstruction can produce extremely dense meshes with far more triangles
than are needed for downstream use. This node applies a decimation algorithm to reduce
the number of faces while preserving the overall shape and important geometric features.
It processes multiple input meshes in parallel using a range-based parallelization scheme.
"""

    commandLine = "aliceVision_lidarDecimating {allParams}"

    size = desc.StaticNodeSize(10)
    parallelization = desc.Parallelization(blockSize=1)
    commandLineRange = "--rangeStart {rangeStart} --rangeSize {rangeFullSize}"

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    category = "Dense Reconstruction"
    inputs = [
        desc.File(
            name="input",
            label="Input JSON",
            description="Input JSON file with description of inputs.",
            value="",
        ),
        desc.FloatParam(
            name="errorLimit",
            label="Error Limit",
            description="Maximal distance (in meters) allowed.",
            value=0.001,
            range=(0.0, 1.0, 0.001),
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
            label="Sub-Meshes Directory",
            description="Output directory for sub-meshes.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="outputJson",
            label="Scene Description",
            description="Output scene description.",
            value="{nodeCacheFolder}/scene.json",
        ),
    ]
