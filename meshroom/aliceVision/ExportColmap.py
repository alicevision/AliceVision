__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL



class ExportColmap(desc.CommandLineNode):
    commandLine = "aliceVision_exportColmap {allParams}"

    category = "Export"
    documentation = """
        Export an AliceVision SfMData to a Colmap scene.

        Creates the folder structure and scene files (cameras.txt, images.txt,
        points3D.txt) that can be used for running a Colmap MVS step.
    """

    inputs = [
        desc.File(
            name="input",
            label="Input",
            description="SfMData file.",
            value="",
        ),
        desc.BoolParam(
            name="copyImages",
            label="Copy Images",
            description="Copy original images to the Colmap folder.\n",
            value=False,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
            exclusive=True,
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="Output",
            description="Base path where the folder structure is created, holding the "
                        "cameras.txt, images.txt and points3D.txt files.",
            value="{nodeCacheFolder}",
        ),
    ]
