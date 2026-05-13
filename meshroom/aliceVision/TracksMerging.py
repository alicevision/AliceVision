__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL

import os.path


class TracksMerging(desc.AVCommandLineNode):
    """
Merge multiple feature track files into a single unified track file.

When feature tracks have been computed independently for different subsets of images
(e.g., for parallel processing or multi-pass matching), this node combines all the
individual track files into one. Tracks that span the same feature across different
files are reconciled to maintain consistency. The merged track file is required as
input by several SfM nodes.
"""

    commandLine = "aliceVision_tracksMerging {allParams}"

    category = "Utils"
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
            label="Output Track File",
            description="Path to the output track file",
            value="{nodeCacheFolder}/tracks.json",
        )
    ]
    