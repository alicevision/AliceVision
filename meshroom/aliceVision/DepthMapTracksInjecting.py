__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class DepthMapTracksInjecting(desc.AVCommandLineNode):
    '''
Inject depth information from depth maps into feature tracks.

For each feature track, the corresponding depth value from the associated depth map is
looked up and stored alongside the 2D observation. This enriched track information can
improve the accuracy of downstream triangulation and bundle adjustment steps, as the
depth constraint provides an additional per-observation measurement.
'''

    commandLine = 'aliceVision_depthmapTracksInjecting {allParams}'

    category = 'Utils'
    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
        ),
        desc.File(
            name="depthSource",
            label="Depth Source",
            description="Directory containing depthMaps",
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
            label="Output Tracks File",
            description="Output Tracks File with updated depth",
            value="{nodeCacheFolder}/tracks.json",
        ),
    ]
