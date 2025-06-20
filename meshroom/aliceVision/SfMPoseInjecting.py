__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json

class SfMPoseInjecting(desc.AVCommandLineNode):

    commandLine = "aliceVision_sfmPoseInjecting {allParams}"
    size = desc.DynamicNodeSize("input")
    
    category = "Utils"
    documentation = """
Use a JSON file to inject poses inside the SfMData.

[
    {
        'frame_no': 45,
        'rx': 1.2,
        'ry': -0.3,
        'rz': 0.1
        'tx': 5,
        'ty': 6,
        'tz': -2
    }
]

or

[
    {
        'path': "image_filename",
        'rx': 1.2,
        'ry': -0.3,
        'rz': 0.1
        'tx': 5,
        'ty': 6,
        'tz': -2
    }
]

or

[
    {
        'path': "/path/to/image",
        'rx': 1.2,
        'ry': -0.3,
        'rz': 0.1
        'tx': 5,
        'ty': 6,
        'tz': -2
    }
]


frame_no is the detected frameId which is set by the number found in the image filename.

Let's say you have a point with coordinates in the *camera frame*. The coordinates in the common world frame are given by the rotation matrix world_R_camera and the translation vector world_t_camera such that 

worldFramePoint = world_R_camera * cameraFramePoint + world_t_camera

world_t_camera is defined by the triplet [tx, ty, tz] in the json file
world_R_camera is defined by the triplet [rx, ry, rz] (in degrees) in the json file.

The matrix world_R_camera is built from the triplet, transformed using a function R depending on rotationFormat.

If rotationFormat is EulerZXY, then world_R_camera = R_y(ry) * R_x(rx) * R_z(rz)
Where R_x(rx) is the rotation along the x axis of rx degrees, R_y(ry) is the rotation along the y axis of ry degrees, R_z(rz) is the rotation along the z axis of rz degrees. It is the ZXY euler representation.

Meshroom assumes that the axis x, y and z of the geometric frame in which you define the rotation and the translation is in Right hand coordinates where X points to the right, Y points downward, and Z points away from the camera. But you can change this using geometricFrame. 

"""

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
            description="Input JSON file containing the poses.",
            value="",
        ),
        desc.ChoiceParam(
            name="rotationFormat",
            label="Rotation Format",
            description="Defines the rotation format for the input poses:\n"
                        " - EulerZXY: Euler rotation in degrees (Y*X*Z)",
            values=["EulerZXY"],
            value="EulerZXY",
        ),
        desc.ChoiceParam(
            name="geometricFrame",
            label="Geometric Frame",
            description="Defines the geometric frame for the input poses:\n"
                        " - RHXrYbZf : Right hand coordinates, X right, Y down, Z far"
                        " - RHXrYtZb : Right hand coordinates, X right, Y up, Z behind",
            values=["RHXrYbZf", "RHXrYtZb"],
            value="RHXrYbZf",
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
