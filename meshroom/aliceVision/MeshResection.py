__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class MeshResection(desc.CommandLineNode):
    
    category = "Utils"

    commandLine = 'aliceVision_meshResection {allParams}'
    size = desc.DynamicNodeSize('input')

    inputs = [
        desc.File(
            name="input",
            label="SfmData",
            description="SfMData file input",
            exposed=True,
            value=""
        ),
        desc.File(
            name="mesh",
            label="Reference Mesh",
            description="Reference mesh file input",
            exposed=True,
            value="",

            commandLineGroup = ''
        ),
        desc.ShapeList(
            name="observations",
            label="Observations",
            description="Keyable observations (3D + 2D).",

            shape=desc.SurveyPoint(
                name="point",
                label="Point",
                description="A 3d+2d point",
                keyable=True,
                keyType="viewId"
            ),

            commandLineGroup = ''
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
            name="json",
            label="Shape Json",
            description="Shapes output",
            semantic="shapeFile",
            value="{nodeCacheFolder}/output.json",
        ),
        desc.File(
            name="output",
            label="Output",
            description="SfMData file output",
            value="{nodeCacheFolder}/output.usda",
        )
    ]

    def preprocess(self, node):

        import json
        liste = [item.getShapeAsDict() for item in node.observations]

        with open(node.json.value, "w") as f:
            json.dump(liste, f)

