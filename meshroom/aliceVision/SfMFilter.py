__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMFilter(desc.CommandLineNode):
    commandLine = "aliceVision_sfmFilter {allParams}"
    category = "Utils"
    documentation = """This node allows select views from SfMData file using a regular expresion."""

    inputs = [
        desc.File(
            name="inputFile",
            label="Input File",
            description="SfMData file.",
            value="",
        ),
        desc.StringParam(
            name="fileMatchingPattern",
            label="File Matching Pattern",
            description="Matching regular expression.\n"
                        "You should capture specific parts of the filepath with parentheses to define matching elements.\n"
                        "Some examples of patterns:\n"
                        " - Match the filename without extension (default value): "
                        r'".*\/(.*?)\.\w{3}"' + "\n"
                        " - Match the filename suffix after \"_\": "
                        r'".*\/.*(_.*?\.\w{3})"' + "\n"
                        " - Match the filename prefix before \"_\": "
                        r'".*\/(.*?)_.*\.\w{3}"',
            value=r'.*\/(.*?)\.\w{3}',
        ),
        desc.BoolParam(
            name="prunePoses",
            label="Remove unselected poses",
            description="Set to True to prune poses related to views that are not selected.",
            value=False,
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
            name="outputSfMData_selected",
            label="Selected SfMData",
            description="Output SfMData file containing selected views.",
            value="{nodeCacheFolder}/selectedSfmData.sfm",
        ),
        desc.File(
            name="outputSfMData_unselected",
            label="Unselected SfMData",
            description="Output SfMData file containing remaining views.",
            value="{nodeCacheFolder}/unselectedSfmData.sfm",
        ),
    ]
