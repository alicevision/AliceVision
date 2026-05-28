__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SphereDetection(desc.CommandLineNode):
    """
Detect spheres in pictures. These spheres will be used for lighting calibration.
Spheres can be automatically detected or manually defined in the interface.
"""

    commandLine = "aliceVision_sphereDetection {allParams}"
    category = "Photometric Stereo"
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="modelPath",
            label="Detection Network",
            description="Deep learning network for automatic calibration sphere detection.",
            value="${ALICEVISION_SPHERE_DETECTION_MODEL}",
        ),
        desc.BoolParam(
            name="autoDetect",
            label="Automatic Sphere Detection",
            description="Automatic detection of calibration spheres.",
            value=False,
        ),
        desc.Circle(
            name="sphereShape",
            label="Sphere Shape",
            description="The shape of the calibration sphere for every image.",
            enabled=lambda node: not node.autoDetect.value,
            commandLineGroup=lambda node: None if node.sphereFile.value else "allParams",
            keyable=True,
            keyType="viewId",
        ),
        desc.File(
            name="sphereFile",
            label="Sphere Shape File",
            description="An input JSON file containing the shapes for every image. If provided, "
                        "the shapes provided with \"Sphere Shape\" will be ignored.",
            semantic="shapeFile",
            value="",
            enabled=lambda node: not node.autoDetect.value,
            commandLineGroup=lambda node: None if not node.sphereFile.value else "allParams",
        ),
        desc.BoolParam(
            name="fillMissingSpheres",
            label="Fill Missing Spheres",
            description="Checked if a sphere position is to be written as detected although it "
                        "was not provided. In that case, the position of the last known sphere "
                        "will be used.",
            value=False,
            enabled=lambda node: not node.autoDetect.value,
        ),
        desc.FloatParam(
            name="minScore",
            label="Minimum Score",
            description="Minimum score for the detection.",
            value=0.0,
            range=(0.0, 50.0, 0.01),
            enabled=lambda node: node.autoDetect.value,
            advanced=True,
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
            label="Output Path",
            description="Sphere detection information will be written here.",
            semantic="shapeFile",
            value="{nodeCacheFolder}/detection.json",
        )
    ]
