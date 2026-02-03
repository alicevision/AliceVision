__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMRigApplying(desc.Node):

    size = desc.DynamicNodeSize("input")
    category = "Utils"
    documentation = """
    Assume the input file is a non calibrated rig with only one pose but N views
    Assume the rig calibration file is a calibrated rig with one or more pose but N sub-poses
    Assume the number of intrinsics is the same and the intrinsics ids are equals.
    Copy information for the rig calibration file to the input file such that the poses of the 
    input file are set to the sub-pose of the calibrated rig.
    """

    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="rig",
            label="Rig SfMData",
            description="Calibrated Rig SfMData file.",
            value="",
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        )
    ]

    outputs = [
        desc.File(
            name="output",
            label="SfM File",
            description="Path to the output SfM file.",
            value="{nodeCacheFolder}/sfmData.sfm",
        )
    ]

    def processChunk(self, chunk):

        from pyalicevision import sfmData as avsfmdata
        from pyalicevision import sfmDataIO as avsfmdataio
        import logging
        import numpy as np

        logging.getLogger().setLevel(chunk.node.verboseLevel.value.upper())

        # Load the input SfmData
        dataInput = avsfmdata.SfMData()
        ret = avsfmdataio.load(dataInput, chunk.node.input.value, avsfmdataio.ALL)
        if not ret:
            logging.error(f"Can't open sfmData file at {chunk.node.input.value}")
            raise RuntimeError()

        logging.info(f"Opened sfmData at {chunk.node.input.value}")

        # Load the sfmData containing the calibrated rig
        dataRig = avsfmdata.SfMData()
        ret = avsfmdataio.load(dataRig, chunk.node.rig.evalValue, avsfmdataio.ALL)
        if not ret:
            logging.error(f"Can't open sfmData file at {chunk.node.rig.evalValue}")
            raise RuntimeError()

        logging.info(f"Opened sfmData at {chunk.node.rig.evalValue}")


        intrinsicsInput = dataInput.getIntrinsics()
        intrinsicsRig = dataRig.getIntrinsics()
        if len(intrinsicsInput) != len(intrinsicsRig):
            logging.error("Incompatible number of intrinsics")
            raise RuntimeError()

        # Copy intrinsics from calibrated rig
        intrinsicsInput.clear()
        for id, item in intrinsicsRig.items():
            intrinsicsInput[id] = item


        rigs = dataRig.getRigs()
        if len(rigs) != 1:
            logging.error("Invalid number of rigs")
            raise RuntimeError()

        # Get first rig
        rig = next(iter(rigs.values()))
        if rig.getNbSubPoses() != len(dataInput.getViews()):
            logging.error("Invalid number of views")
            raise RuntimeError()

        if not rig.isFullyCalibrated():
            logging.error("Rig is not calibrated")
            raise RuntimeError()


        # Copy the relative pose contained in the rig
        # As the pose of the view
        posesInput = dataInput.getPoses()
        for id, v in dataInput.getViews().items():
            subPose = rig.getSubPose(v.getSubPoseId())
            pose = avsfmdata.CameraPose(subPose.pose)

            posesInput[id] = pose

            v.setPoseId(id)
            v.setRigAndSubPoseId(avsfmdata.UndefinedIndexT, avsfmdata.UndefinedIndexT)

        # Save the sfmData
        avsfmdataio.save(dataInput, chunk.node.output.value, avsfmdataio.ALL)
        
