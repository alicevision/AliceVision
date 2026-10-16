__version__ = "2.2"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import os.path


class SfMTransfer(desc.AVCommandLineNode):
    """
Transfer camera poses, intrinsics, landmarks and survey points from a reference SfM scene onto
another (destination) SfM scene.

The node matches views between the destination SfMData ('Input') and the reference SfMData
('Reference') using one of several strategies, then copies the resolved data of the reference
scene into the destination scene. It is useful for propagating a high-quality reconstruction onto
a different but related set of images, or for applying externally computed camera parameters.

The views themselves are never added nor removed: the output contains exactly the views of the
destination scene. Image paths, metadata, image groups and rigs that do not already exist in the
destination are never created. Only the components described below can be modified.


1. View matching (methods 'from_viewid', 'from_filepath' and 'from_metadata')

- from_viewid: a destination view is matched with the reference view having the same view ID.
- from_filepath: the regular expression 'File Matching Pattern' is applied to the image path of
  every view. The parts captured by the parentheses are concatenated to build a key, and views with
  the same key in both scenes are matched. If the pattern is empty, the full image path is the key.
  A view whose path does not match the pattern gets an empty key and is never matched.
- from_metadata: the values of the metadata listed in 'Metadata Matching List' are concatenated to
  build a key, and views with the same key in both scenes are matched. A missing metadata simply
  contributes nothing to the key, so make sure the chosen metadata really identify each image.
  A view with none of the listed metadata gets an empty key and is never matched.

For 'from_filepath' and 'from_metadata', a key shared by several views of the same scene is
ambiguous: all these views are discarded, so that only one-to-one matches are kept.
If no matching view is found at all, the node fails.

A matched pair is only used for the pose and intrinsic transfers if the reference view is
reconstructed, i.e. it has a defined pose and a defined intrinsic (and, for a rig-dependent view,
an initialized rig sub-pose). Matched pairs whose reference view is not reconstructed leave the
destination view pose and intrinsic untouched.

Destination views that are not matched keep all their data untouched.


2. Poses ('Poses' option)

For each matched pair with a reconstructed reference view, the resection ID of the reference view
is copied onto the destination view, then:

- If the reference view is part of a rig with a rig-dependent pose, and the destination view is
  part of a rig which exists in the destination scene, the rig structure is kept:
    * the destination view is attached to the pose of its rig frame (pose ID computed from its rig
      ID and frame ID) and its pose becomes rig-dependent;
    * the rig frame pose is overwritten with the reference rig frame pose;
    * only the sub-pose of this destination view is overwritten with the sub-pose of the reference
      view (the list of sub-poses of the destination rig is extended if required). The other
      sub-poses of the destination rig are left untouched.
  If views of different reference frames are matched with views of the same destination frame,
  a warning is emitted and the last transferred frame pose is kept.

- Otherwise, the absolute pose of the reference view is copied (for a rig-dependent reference view,
  the composition of its rig frame pose and its sub-pose). If the destination view was part of a rig
  with a rig-dependent pose, it is detached from the rig pose: it gets its own independent pose,
  whose pose ID is its view ID. Any existing pose with the target pose ID is overwritten.

Poses of the destination scene which are no longer used by any view after the transfer are removed.


3. Intrinsics ('Intrinsics' option)

For each matched pair with a reconstructed reference view, the reference intrinsic is copied
(cloned, so the camera model may change) onto the destination:

- The first time a destination intrinsic is matched with a given reference intrinsic, the
  destination intrinsic is overwritten in place, keeping its intrinsic ID. As a consequence,
  destination views sharing this intrinsic but not matched (or matched with a non-reconstructed
  reference view) also see the new intrinsic values.
- If views of the same destination intrinsic are matched with another reference intrinsic, a new
  destination intrinsic (new ID, after the highest existing one) is created holding a copy of it,
  and these views are moved onto it. A destination intrinsic is therefore split rather than having
  views with different reference calibrations share it.
- Destination intrinsics are never merged, even if they are matched with the same reference intrinsic.
- If the destination view refers to an intrinsic ID that does not exist, a new intrinsic is created.

Intrinsics of the destination scene which are no longer used by any view are removed.


4. Landmarks ('Landmarks' option)

- If the reference scene has no landmarks, the destination landmarks are kept as they are.
- Otherwise, the destination landmarks are entirely replaced by the reference landmarks (existing
  destination landmarks are discarded). Each reference landmark keeps its ID, position, color and
  descriptor type; its observations are re-indexed onto the matched destination view IDs.
  Observations in reference views that are not matched are dropped, and landmarks left without any
  observation are dropped.
Note that landmark transfer does not check if the reference views are reconstructed.


5. Survey points ('Survey Points' option)

Survey points are stored per view. For each matched pair, if the reference view has survey points,
they replace the survey points of the destination view. Destination views without reference survey
points keep their existing ones.


6. Methods 'from_poseid' and 'from_intrinsicid'

These methods do not match views and ignore the 'Poses', 'Intrinsics', 'Landmarks' and
'Survey Points' options (at least one of them must still be enabled):

- from_poseid: for each destination view whose pose ID exists in the reference scene, the
  destination pose with this ID is overwritten by the reference pose. Rigs, intrinsics,
  landmarks and survey points are left untouched.
- from_intrinsicid: each destination intrinsic whose ID exists in the reference scene is replaced
  by a copy of the reference intrinsic. Nothing else is modified.


7. Cleanup and outputs

Whatever the method, unused poses, landmarks without observations and unused intrinsics are
removed from the destination scene before saving. The full scene is written to 'SfMData', and the
views, poses and intrinsics only are written to 'Poses'.
"""

    # The transfer options only apply to the methods matching views
    matchesViews = lambda node: node.method.value not in ["from_poseid", "from_intrinsicid"]

    commandLine = "aliceVision_sfmTransfer {allParams}"
    size = desc.DynamicNodeSize("input")

    category = "Utils"
    inputs = [
        desc.File(
            name="input",
            label="Input",
            description="Path to the destination SfMData file. This is the SfM scene onto which the camera poses, intrinsics, and landmarks will be transferred.",
            value="",
        ),
        desc.File(
            name="reference",
            label="Reference",
            description="Path to the reference SfMData file used to retrieve resolved poses, intrinsics and landmarks.",
            value="",
        ),
        desc.ChoiceParam(
            name="method",
            label="Matching Method",
            description="Matching Method:\n"
                " - from_viewid: Match views with same view ID.\n"
                " - from_filepath: Match views with a filepath matching, using 'fileMatchingPattern'.\n"
                " - from_metadata: Match views with matching metadata, using 'metadataMatchingList'.\n"
                " - from_poseid: Match poses with the same pose ID. Only the poses will be updated.\n"
                " - from_intrinsicid: Match intrinsics with the same intrinsic ID. Only the intrinsics will be updated.\n"
                "The 'from_poseid' and 'from_intrinsicid' methods ignore the transfer options.",
            value="from_viewid",
            values=["from_viewid", "from_filepath", "from_metadata", "from_poseid", "from_intrinsicid"],
        ),
        desc.StringParam(
            name="fileMatchingPattern",
            label="File Matching Pattern",
            description="Matching regular expression for the 'from_filepath' method.\n"
                        "You should capture specific parts of the filepath with parentheses to define matching elements.\n"
                        "If empty, the full image path is used. Values shared by several views of a scene are ignored.\n"
                        "Some examples of patterns:\n"
                        " - Match the filename without extension (default value): "
                        r'".*\/(.*?)\.\w{3}"' + "\n"
                        " - Match the filename suffix after \"_\": "
                        r'".*\/.*(_.*?\.\w{3})"' + "\n"
                        " - Match the filename prefix before \"_\": "
                        r'".*\/(.*?)_.*\.\w{3}"',
            value=r'.*\/(.*?)\.\w{3}',
        ),
        desc.ListAttribute(
            elementDesc=desc.File(
                name="metadataMatching",
                label="Metadata",
                description="Metadata that should match to create correspondences.",
                value="",
            ),
            name="metadataMatchingList",
            label="Metadata Matching List",
            description="List of metadata that should match to create the correspondences.\n"
                        "If the list is empty, the default value will be used:\n"
                        "['Make', 'Model', 'Exif:BodySerialNumber', 'Exif:LensSerialNumber'].",
        ),
        desc.BoolParam(
            name="transferPoses",
            label="Poses",
            description="Transfer poses of the reconstructed reference views onto the matched views.\n"
                        "The rig structure is kept if both views are rig-dependent, otherwise the view gets an independent pose.",
            value=True,
            enabled=matchesViews,
        ),
        desc.BoolParam(
            name="transferIntrinsics",
            label="Intrinsics",
            description="Transfer cameras intrinsics of the reconstructed reference views onto the matched views.\n"
                        "A destination intrinsic is overwritten, or split if its views match different reference intrinsics.",
            value=True,
            enabled=matchesViews,
        ),
        desc.BoolParam(
            name="transferLandmarks",
            label="Landmarks",
            description="Replace the destination landmarks by the reference landmarks observed in matched views.\n"
                        "Nothing is done if the reference scene has no landmarks.",
            value=True,
            enabled=matchesViews,
        ),
        desc.BoolParam(
            name="transferSurveyPoints",
            label="Survey Points",
            description="Replace the survey points of each matched view by the ones of its reference view, if any.",
            value=True,
            enabled=matchesViews,
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
            description="Path to the output SfM point cloud file (in SfMData format).",
            value=lambda attr: "{nodeCacheFolder}/" + (os.path.splitext(os.path.basename(attr.node.input.value))[0] or "sfmData") + ".usda",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        ),
    ]
