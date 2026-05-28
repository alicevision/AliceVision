__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

class LidarMerging(desc.AVCommandLineNode):
    """
Merge multiple meshes reconstructed from LiDAR data into a single unified mesh.

After individual LiDAR point cloud patches have been independently meshed (by LidarMeshing)
and optionally decimated (by LidarDecimating), this node combines all the partial meshes
into one continuous mesh. Overlapping regions between adjacent patches are handled to
produce a seamless result.
"""

    commandLine = "aliceVision_lidarMerging {allParams}"

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
            label="Mesh Path Output",
            description="Output directory for mesh.",
            value="{nodeCacheFolder}/output.obj",
        ),
    ]
