__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL
from pyalicevision import parallelization as avpar

class MaskProcessingNodeSize(desc.DynamicNodeSize):
    """
    MaskProcessingNodeSize expresses a dependency to multiple input attributess to define
    the size of a Node in terms of individual tasks for parallelization.
    """
    def __init__(self, param):
        self._params = param

    def __call__(self, node):

        size = 0

        for input in node.attribute(self._params).value:
            paramName = input.fullName
            param = node.attribute(paramName)
            if param.isLink:
                size = max(size, param.inputLink.node.size)

        return size


class MaskProcessing(desc.AVCommandLineNode):
    """
Perform Boolean and morphological operations on sets of binary masks.

Given multiple directories of binary masks (where corresponding masks share the same filename),
this node can combine them using logical operations (AND, OR, XOR). This is useful for combining
masks produced by different segmentations.
"""

    commandLine = "aliceVision_maskProcessing {allParams}"
    size = MaskProcessingNodeSize("inputs")

    category = "Utils"
    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input Directory",
                description="A directory with a set of mask.",
                value="",
            ),
            name="inputs",
            label="Input Directories",
            description="A set of directories containing masks with the same names.\n" 
            "Any entry (except the first one) may be an image path. In this case, this mask will be used as an operand for each entry of the first directory.",
            exposed=True,
        ),
        desc.ChoiceParam(
            name="operator",
            label="Operator",
            description="Operator: Binary operator\n"
                        "OR: applies binary OR between all the masks\n"
                        "AND: applies binary AND between all the masks\n"
                        "NOT: applies binary NOT to the first mask in the list\n",
            value="and",
            values=["or", "and", "not"],
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
            label="Output",
            description="Path to the output directory.",
            value="{nodeCacheFolder}",
        ),
        desc.File(
            name="masks",
            label="Masks",
            description="Processed segmentation masks.",
            semantic="imageList",
            value= "{nodeCacheFolder}/*.exr",
            commandLineGroup="",
        ),
    ]
