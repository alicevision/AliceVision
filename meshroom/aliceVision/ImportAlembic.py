__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ImportAlembic(desc.AVCommandLineNode):
    commandLine = "aliceVision_importAlembic {allParams}"

    category = "Utils"
    documentation = """
Import an external Alembic file that does not follow the SfMData convention, and populates a valid SfMData with its camera poses.
"""

    inputs = [
        desc.File(
            name="input",
            label="Alembic File",
            description="The external Alembic file to import.",
            value="",
        ),
        desc.File(
            name="imagesDir",
            label="Images Directory",
            description="Directory containing the images.",
            value="",
        ),
        desc.ChoiceParam(
            name="extension",
            label="Images Extension",
            description="File extension for the images in the directory to be taken into account.",
            value="exr",
            values=["exr", "jpg", "png"],
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            value="info",
            values=VERBOSE_LEVEL,
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="SfMData",
            description="SfMData file populated with the camera poses from the external Alembic file.",
            value="{nodeCacheFolder}/importedAbc.sfm",
        ),
    ]
