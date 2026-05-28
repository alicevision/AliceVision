__version__ = "2.4"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMExpanding(desc.AVCommandLineNode):
    """
Expand an incremental Structure-from-Motion reconstruction by localizing additional cameras.

Starting from an initialized SfMData (produced by SfmBootstrapping), this node iteratively
adds new views to the reconstruction. For each candidate view, it localizes the camera by
finding 2D-3D correspondences between tracked features and existing 3D landmarks (resectioning),
then triangulates new landmarks that become visible from the newly added cameras.

The process continues until all views have been processed or no further views can be added.
Bundle adjustment is performed periodically to refine all camera poses and 3D point positions.
"""

    commandLine = "aliceVision_sfmExpanding {allParams}"
    size = desc.DynamicNodeSize("input")

    cpu = desc.Level.INTENSIVE
    ram = desc.Level.INTENSIVE

    category = "Sparse Reconstruction"
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
        desc.File(
            name="meshFilename",
            label="Mesh File",
            description="Mesh file (*.obj).",
            value="",
        ),
        desc.IntParam(
            name="localizerEstimatorMaxIterations",
            label="Localizer Max Ransac Iterations",
            description="Maximum number of iterations allowed in the Ransac step.",
            value=50000,
            range=(1, 100000, 1),
            advanced=True,
        ),
        desc.FloatParam(
            name="localizerEstimatorError",
            label="Localizer Max Ransac Error",
            description="Maximum error (in pixels) allowed for camera localization (resectioning).\n"
                        "If set to 0, it will select a threshold according to the localizer estimator used\n"
                        "(if ACRansac, it will analyze the input data to select the optimal value).",
            value=0.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
        ),
        desc.BoolParam(
            name="enableDepthPrior",
            label="Use Depth Prior",
            description="If available in the tracks, use the depth prior to help the structure estimation.",
            value=True,
        ),
        desc.BoolParam(
            name="ignoreMultiviewOnPrior",
            label="Ignore Multiview On Prior",
            description="Favour the prior based 3d reconstruction over the multiview reconstruction.",
            value=False,
        ),
        desc.BoolParam(
            name="lockScenePreviouslyReconstructed",
            label="Lock Previously Reconstructed Scene",
            description="Lock previously reconstructed poses and intrinsics.\n"
                        "This option is useful for SfM augmentation.",
            value=False,
        ),
        desc.BoolParam(
            name="useLocalBA",
            label="Local Bundle Adjustment",
            description="It reduces the reconstruction time, especially for large datasets (500+ images),\n"
                        "by avoiding computation of the Bundle Adjustment on areas that are not changing.",
            value=True,
        ),
        desc.BoolParam(
            name="useTemporalConstraint",
            label="Temporal Constraint",
            description="Adds a temporal smoothness constraint to the bundle adjustment.",
            value=False,
        ),
        desc.FloatParam(
            name="tscPositionWeight",
            label="Temporal Constraint Position Weight",
            description="Controls the weight of the temporal constraint applied to camera positions. Higher values enforce smoother camera path.",
            value=10.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscOrientationWeight",
            label="Temporal Constraint Orientation Weight",
            description="Controls the weight of the temporal constraint applied to camera orientations. Higher values enforce smoother camera rotation.",
            value=10.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC0positionWeight",
            label="Temporal Constraint C0 Position Weight",
            description="Controls the weight of the continuity constraint on camera positions in the temporal constraint. Higher values enforce smoother transitions in position, reducing abrupt changes of position.",
            value=0.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC1positionWeight",
            label="Temporal Constraint C1 Position Weight",
            description="Controls the weight of the first derivative of camera position in the temporal constraint. Higher values enforce continuity of the camera velocity, reducing abrupt changes of velocity.",
            value=1.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC2positionWeight",
            label="Temporal Constraint C2 Position Weight",
            description="Controls the weight of the second derivative of camera position in the temporal constraint. Higher values enforce continuity of the camera acceleration, reducing abrupt changes of acceleration.",
            value=1.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC0orientationWeight",
            label="Temporal Constraint C0 Orientation Weight",
            description="Controls the weight of the continuity constraint on camera orientation in the temporal constraint. Higher values enforce smoother transitions, reducing abrupt changes of the camera orientation.",
            value=0.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC1orientationWeight",
            label="Temporal Constraint C1 Orientation Weight",
            description="Controls the weight of the first derivative of camera orientation in the temporal constraint. Higher values enforce continuity of the rotation velocity, reducing abrupt changes of rotation velocity.",
            value=1.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscC2orientationWeight",
            label="Temporal Constraint C2 Orientation Weight",
            description="Controls the weight of the second derivative of camera orientation in the temporal constraint. Higher values enforce continuity of the rotation acceleration, reducing abrupt changes of rotation acceleration.",
            value=1.0,
            range=(0.0, 1.0, 0.01),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscLand2ViewsRegWeight",
            label="Scene Scale Based Regularization Weight",
            description="Controls the strength of a regularization applied to the temporal constraint, encouraging the mean distance between the landmarks and the views to remain constant.",
            value=0.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.FloatParam(
            name="tscTrajLengthRegWeight",
            label="Trajectory Length Based Regularization Weight",
            description="Controls the strength of a regularization applied to the temporal constraint, encouraging the trajectory length to remain constant.",
            value=0.0,
            range=(0.0, 100.0, 0.1),
            advanced=True,
            enabled=lambda node: node.useTemporalConstraint.value,
        ),
        desc.IntParam(
            name="localBAGraphDistance",
            label="LocalBA Graph Distance",
            description="Graph-distance limit to define the active region in the Local Bundle Adjustment strategy.",
            value=1,
            range=(2, 10, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="nbFirstUnstableCameras",
            label="First Unstable Cameras Nb",
            description="Number of cameras for which the bundle adjustment is performed every single time a camera is added.\n"
                        "This leads to more stable results while computations are not too expensive, as there is little data.\n"
                        "Past this number, the bundle adjustment will only be performed once for N added cameras.",
            value=30,
            range=(0, 100, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="maxImagesPerGroup",
            label="Max Images Per Group",
            description="Maximum number of cameras that can be added before the bundle adjustment has to be performed again.\n"
                        "This prevents adding too much data at once without performing the bundle adjustment.",
            value=30,
            range=(0, 100, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="bundleAdjustmentMaxOutliers",
            label="Max Nb Of Outliers After BA",
            description="Threshold for the maximum number of outliers allowed at the end of a bundle adjustment iteration.\n"
                        "Using a negative value for this threshold will disable BA iterations.",
            value=50,
            range=(-1, 1000, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="weakResectionSize",
            label="Weak resection inliers count",
            description="When adding a view during the expansion process, we compute the pose. If the inliers count\n"
                        "Is less than this value, the resection is considered weak. If not all views in the batch \n"
                        "are weak, then the weak views are put back in the list of views to estimate again",
            value=100,
            range=(1, 1000, 1),
            advanced=True,
        ),
        desc.IntParam(
            name="minNumberOfObservationsForTriangulation",
            label="Min Observations For Triangulation",
            description="Minimum number of observations to triangulate a point.\n"
                        "Setting it to 3 (or more) reduces drastically the noise in the point cloud,\n"
                        "but the number of final poses is a little bit reduced\n"
                        "(from 1.5% to 11% on the tested datasets).",
            value=2,
            range=(2, 10, 1),
            advanced=True,
        ),
        desc.FloatParam(
            name="minAngleForTriangulation",
            label="Min Angle For Triangulation",
            description="Minimum angle for triangulation.",
            value=3.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="minAngleForLandmark",
            label="Min Angle For Landmark",
            description="Minimum angle for landmark.",
            value=2.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="maxReprojectionError",
            label="Max Reprojection Error",
            description="Maximum reprojection error in the bundle verification step.",
            value=4.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.FloatParam(
            name="maxTriangulationError",
            label="Max Triangulation Error",
            description="Maximum reprojection error in the triangulation process.",
            value=8.0,
            range=(0.1, 10.0, 0.1),
            advanced=True,
        ),
        desc.BoolParam(
            name="lockAllIntrinsics",
            label="Lock All Intrinsic Camera Parameters",
            description="Force to keep all the intrinsic parameters of the cameras (focal length, \n"
                        "principal point, distortion if any) constant during the reconstruction.\n"
                        "This may be helpful if the input cameras are already fully calibrated.",
            value=False,
        ),
        desc.BoolParam(
            name="enableStructureRefinement",
            label="Enable Structure Refinement",
            description="Bundle adjustment will try to optimize the landmarks positions.",
            value=True,
        ),
        desc.IntParam(
            name="minNbCamerasToRefinePrincipalPoint",
            label="Min Nb Cameras To Refine Principal Point",
            description="Minimum number of cameras to refine the principal point of the cameras (one of the intrinsic parameters of the camera).\n"
                        "If we do not have enough cameras, the principal point is considered to be in the center of the image.\n"
                        "If minNbCamerasToRefinePrincipalPoint <= 0, the principal point is never refined."
                        "If minNbCamerasToRefinePrincipalPoint is set to 1, the principal point is always refined.",
            value=3,
            range=(0, 20, 1),
            advanced=True,
        ),
        desc.BoolParam(
            name="useRigConstraint",
            label="Use Rig Constraint",
            description="Enable/Disable rig constraint.",
            value=True,
            advanced=True,
        ),
        desc.IntParam(
            name="rigMinNbCamerasForCalibration",
            label="Min Nb Cameras For Rig Calibration",
            description="Minimum number of cameras to start the calibration of the rig.",
            value=20,
            range=(1, 50, 1),
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
            label="SfMData",
            description="Path to the output SfMData file.",
            value="{nodeCacheFolder}/sfmExpanded.abc",
        ),
        desc.File(
            name="outputViewsAndPoses",
            label="Views And Poses",
            description="Path to the output SfMData file with cameras (views and poses).",
            value="{nodeCacheFolder}/cameras.sfm",
        )
    ]

    def onUseTemporalConstraintChanged(self, node):
        if node.useTemporalConstraint.value:
            node.useLocalBA.value = False

    def onUseLocalBAChanged(self, node):
        if node.useLocalBA.value:
            node.useTemporalConstraint.value = False
