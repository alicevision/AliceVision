__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

class GeometricFilterFromPoses(desc.AVCommandLineNode):
    '''
Build geometric filters from known camera poses.

For each pair of views with a known pose and intrinsic, a fundamental matrix is computed from the cameras.
The generated filters can be used by GeometricFilterApplying to discard matches whose epipolar distance
is above the user defined threshold.
'''

    commandLine = 'aliceVision_geometricFilterFromPoses {allParams}'
    size = desc.DynamicNodeSize("input")
    
    category = 'Sparse Reconstruction'
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file with known poses and intrinsics.",
            value="",
        ),
        desc.FloatParam(
            name="threshold",
            label="Epipolar Threshold",
            description="Maximum epipolar distance (in pixels) for a match to be considered as an inlier.",
            value=4.0,
            range=(0.1, 50.0, 0.1),
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
            label="Filters Folder",
            description="Path to a folder in which the computed filters are stored.",
            value="{nodeCacheFolder}",
        ),
    ]
