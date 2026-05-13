__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class GlobalPositionEstimating(desc.AVCommandLineNode):
    """
Estimate global camera translations and recover 3D structure from feature tracks.

As part of the Global SfM pipeline, once global rotations have been estimated, this node
solves for the global camera translations and the 3D positions of all scene landmarks.
It formulates the problem as a linear system using the relative translation directions
between image pairs and resolves scale ambiguities using the track observations.
"""

    commandLine = "aliceVision_globalPositionEstimating {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Sparse Reconstruction"
    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="SfMData file.",
            value="",
        ),
        desc.File(
            name="tracksFilename",
            label="Tracks File",
            description="Tracks file.",
            value="",
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
            description="Path to the output SfMData file.",
            value="{nodeCacheFolder}/sfm.abc",
        ),
    ]
