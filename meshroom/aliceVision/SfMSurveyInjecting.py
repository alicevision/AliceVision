__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json

class SfMSurveyInjecting(desc.AVCommandLineNode):
    """
Inject geodetic or physical survey measurements from a JSON file into an SfMData scene.

Survey measurements (e.g., GPS coordinates or physical distance measurements between
known control points) provide absolute scale and orientation constraints. This node reads
a JSON file containing survey data and embeds the measurements as constraints within the
SfMData. These constraints can then be used to geo-register the reconstruction or to
enforce a known metric scale during bundle adjustment.
"""


    commandLine = "aliceVision_sfmSurveyInjecting {allParams}"
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
            name="surveyFilename",
            label="Survey",
            description="Input JSON file containing the survey.",
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
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/sfmData.abc",
        ),
    ]
