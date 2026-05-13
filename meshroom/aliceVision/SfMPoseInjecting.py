__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json
import pathlib

class SfMPoseInjecting(desc.AVCommandLineNode):
    """
Inject external camera pose data from a JSON or Alembic file into an SfMData scene.

This node reads pre-computed camera poses (position and orientation) from an external
file and assigns them to the corresponding views in the SfMData, matching views by frame
number with an optional offset. The injected poses can be locked to prevent them from
being modified by subsequent bundle adjustment steps. This is useful when poses have
been obtained from an IMU, GPS, or another tracking system and should be used as
constraints or initial values for the reconstruction.
"""


    commandLine = "aliceVision_sfmPoseInjecting {allParams}"
    size = desc.DynamicNodeSize("input")
    
    category = "Utils"
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
        desc.IntParam(
            name="offset",
            label="Frame number offset",
            description="Offset to use on the reference file frame number to match the input frame number.",
            value=0,
        ),
        desc.BoolParam(
            name="lockPoses",
            label="Lock Injected Poses",
            description="Do we lock the pose parameters for future refinement ?",
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
            name="output",
            label="SfMData",
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
    ]
