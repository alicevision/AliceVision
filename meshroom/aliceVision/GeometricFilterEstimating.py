__version__ = "1.1"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL
from pyalicevision import parallelization as avpar

class GeometricFilterEstimating(desc.AVCommandLineNode):
    '''
It performs a geometric filtering of the photometric match candidates.
It uses the features positions in the images to make a geometric filtering by using epipolar geometry in an outlier detection framework
called RANSAC (RANdom SAmple Consensus). It randomly selects a small set of feature correspondences and compute the fundamental (or essential) matrix,
then it checks the number of features that validates this model and iterate through the RANSAC framework.

## Online
[https://alicevision.org/#photogrammetry/feature_matching](https://alicevision.org/#photogrammetry/feature_matching)
'''

    commandLine = 'aliceVision_geometricFilterEstimating {allParams}'
    size = avpar.DynamicViewsSize("input")
    parallelization = desc.Parallelization(blockSize=20)
    commandLineRange = '--rangeIteration {rangeIteration} --rangeBlocksCount {rangeBlocksCount}'

    category = 'Sparse Reconstruction'
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.ListAttribute(
            elementDesc=desc.File(
                name="featuresFolder",
                label="Features Folder",
                description="Folder containing some extracted features and descriptors.",
                value="",
            ),
            name="featuresFolders",
            label="Features Folders",
            description="Folder(s) containing the extracted features and descriptors.",
            exposed=True,
        ),
        desc.ListAttribute(
            elementDesc=desc.File(
                name="matchesFolder",
                label="Matches Folder",
                description="Folder containing some matches.",
                value="",
            ),
            name="matchesFolders",
            label="Matches Folders",
            description="Folder(s) in which computed matches are stored.",
            exposed=True,
        ),
        desc.ChoiceParam(
            name="describerTypes",
            label="Describer Types",
            description="Describer types used to describe an image.",
            values=DESCRIBER_TYPES,
            value=["dspsift"],
            exclusive=False,
            joinChar=",",
            exposed=True,
        ),
        desc.IntParam(
            name="maxIteration",
            label="Max Iterations",
            description="Maximum number of iterations allowed in the Ransac step.",
            value=50000,
            range=(1, 100000, 1),
            advanced=True,
        ),
        desc.FloatParam(
            name="geometricError",
            label="Geometric Validation Error",
            description="Maximum error (in pixels) allowed for features matching during geometric verification",
            value=0.0,
            range=(0.0, 10.0, 0.1),
            advanced=True,
        ),
        desc.IntParam(
            name="minMatches",
            label="Min Matches",
            description="Minimum number of matches to accept a pair of images (or 0 to disable limit).",
            value=0,
            range=(0, 10000, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="maxMatches",
            label="Max Matches",
            description="Maximum number of matches to keep.",
            value=0,
            range=(0, 10000, 1),
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
            name="output",
            label="Filters Folder",
            description="Path to a folder in which the computed filters are stored.",
            value="{nodeCacheFolder}",
        ),
    ]
