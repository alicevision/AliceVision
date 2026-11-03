__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class MeshResection(desc.CommandLineNode):
    
    category = "Utils"
    documentation = """
Estimate camera poses by manually aligning views on a reference mesh.

For each view, the user places survey points: every survey point links a 2D location in the image
to a 3D location picked on the reference mesh. The camera pose of the view is then computed by
resection (robust PnP) from these 2D/3D correspondences.

Requirements:
 - Each survey point must be defined both in 2D (picked in the image) and in 3D (picked on the mesh).
   Points defined in only one of the two spaces are ignored.
 - Each view needs at least 4 valid survey points to be localized. Views with fewer points are skipped.
 - The input SfMData must provide intrinsics with an approximate focal length for each view,
   as the focal length is not estimated by this node.

Outputs:
 - An SfMData containing the estimated poses for the localized views, along with their survey points.
 - A JSON file with the survey points, updated with the reprojection of each 3D point
   in the localized views so the alignment error can be inspected.

The node fails if no view could be localized.
"""

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

