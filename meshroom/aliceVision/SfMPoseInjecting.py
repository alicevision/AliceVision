__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json
import pathlib

class SfMPoseInjecting(desc.AVCommandLineNode):

    commandLine = "aliceVision_sfmPoseInjecting {allParams}"
    size = desc.DynamicNodeSize("input")
    
    category = "Utils"
    documentation = """Use a JSON file to inject poses inside the SfMData."""

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="posesFilename",
            label="Poses",
            description="Input file containing the poses (Json or ABC).",
            value="",
        ),
        desc.FloatParam(
            name="framerate",
            label="Frame rate",
            description="Alembic frame rate to compute frame id from time",
            value=24.0,
            range=(10.0, 50.0, 1.0),
            enabled=lambda node: pathlib.Path(node.posesFilename.value).suffix.lower() == ".abc"
        ),
        desc.ChoiceParam(
            name="rotationFormat",
            label="Rotation Format",
            description="Defines the rotation format for the input poses:\n"
                        " - EulerZXY: Euler rotation in degrees (Y*X*Z)",
            values=["EulerZXY"],
            value="EulerZXY",
            enabled=lambda node: pathlib.Path(node.posesFilename.value).suffix.lower() == ".json"
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
            label="SfMData",
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
    ]
