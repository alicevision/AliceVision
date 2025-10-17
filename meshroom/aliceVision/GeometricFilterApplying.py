__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL


class GeometricFilterApplying(desc.AVCommandLineNode):
    commandLine = 'aliceVision_geometricFilterApplying {allParams}'
    size = desc.DynamicNodeSize('input')
    parallelization = desc.Parallelization(blockSize=20)
    commandLineRange = '--rangeIteration {rangeIteration} --rangeBlocksCount {rangeBlocksCount}'

    category = 'Sparse Reconstruction'
    documentation = '''
Apply precomputed transforms to matches to filter geometric matches
'''

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
        desc.File(
            name="filters",
            label="Filters Folder",
            description="Path to a folder in which the computed filters are stored.",
            value="",
            exposed=True
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
            label="Matches Folder",
            description="Path to a folder in which the computed matches are stored.",
            value="{nodeCacheFolder}",
        ),
    ]
