__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ExportAlembic(desc.AVCommandLineNode):
    commandLine = "aliceVision_exportAlembic {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Export"
    documentation = """
Convert cameras from an SfM scene into an animated cameras in Alembic file format.
Based on the input image filenames, it will recognize the input video sequence to create an animated camera.
"""

    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="SfMData file containing a complete SfM.",
            value="",
        ),
        desc.FloatParam(
            name="frameRate",
            label="Camera Frame Rate",
            description="Define the camera's Frames per seconds.",
            value=24.0,
            range=(1.0, 60.0, 1.0),
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
            label="Alembic filename",
            description="Output alembic filename",
            value="{nodeCacheFolder}/animated.abc",
        )
    ]
