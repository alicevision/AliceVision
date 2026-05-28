__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL

import os.path

class MergeNodeSize(desc.DynamicNodeSize):
    """
    MergeNodeSize expresses a dependency to multiple input attributess to define
    the size of a Node in terms of individual tasks for parallelization.
    """
    def __init__(self, param):
        self._params = param

    def __call__(self, node):

        size = 0

        for input in node.attribute(self._params).value:
            paramName = input.fullName
            param = node.attribute(paramName)
            size = size + param.inputLink.node.size

        return size


class SfMMerge(desc.AVCommandLineNode):
    """
Merge two or more SfMData files into a single unified SfMData scene.

All input SfMData files must have disjoint view and landmark identifiers (UIDs);
the node will fail if any UID is shared between inputs to prevent ambiguous merging.
Various merging strategies are available to control how overlapping intrinsics or
poses are handled. This node is typically used to combine independently reconstructed
sub-scenes before a joint bundle adjustment.
"""

    commandLine = "aliceVision_sfmMerge {allParams}"
    size = MergeNodeSize("inputs")

    category = "Utils"
    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input SfmData",
                description="A SfmData file.",
                value="",
            ),
            name="inputs",
            label="Inputs",
            description="Set of SfmData (at least 1 is required).",
            exposed=True,
        ),
        desc.ChoiceParam(
            name="method",
            label="Merge Method",
            description="Merge method:\n"
                        " - simple copy: Straight copy without duplicate management.\n"
                        " - from_landmarks: Align from matched features, try to fuse.\n",
            value="simple_copy",
            values=["simple_copy", 'from_landmarks'],
        ),
        desc.BoolParam(
            name="ignoreDuplicates",
            label="Ignore duplicates",
            description="If disabled, an error will be thrown if a duplicate view or intrinsic is found.",
            enabled=lambda node: node.method.value == "simple_copy",
            value=False,
        ),
        desc.ListAttribute(
            elementDesc=desc.File(
                name="matchesFolder",
                label="Matches Folder",
                description="",
                value="",
            ),
            name="matchesFolders",
            label="Matches Folders",
            description="Folder(s) in which the computed matches are stored.",
        ),
        desc.ChoiceParam(
            name="describerTypes",
            label="Describer Types",
            description="Describer types used to describe an image.",
            values=DESCRIBER_TYPES,
            value=["dspsift"],
            exclusive=False,
            joinChar=",",
        ),
        desc.ChoiceParam(
            name="fileExt",
            label="SfM File Format",
            description="Output SfM file format.",
            value="abc",
            values=["abc", "sfm", "json"],
            commandLineGroup="",  # exclude from command line
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
            label="SfMData", 
            description="Path to the output SfM file (in SfMData format).",
            value="{nodeCacheFolder}/sfmData.{fileExtValue}",
        )
    ]
