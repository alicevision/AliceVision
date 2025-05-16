__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL

import os.path


class TracksMerging(desc.AVCommandLineNode):
    commandLine = 'aliceVision_tracksMerging {allParams}'

    category = 'Utils'
    documentation = '''
Merges multiple track files into one
'''

    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input Track File",
                description="A track file.",
                value="",
            ),
            name="inputs",
            label="Inputs",
            description="Set of track files (at least 1 is required).",
            exposed=True,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        )
    ]

    outputs = [
        desc.File(
            name="output",
            label="Output tracks file", 
            description="Path to the output track file",
            value="{nodeCacheFolder}/tracks.json",
        )
    ]
    