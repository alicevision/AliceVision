__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMPoseFlattening(desc.Node):

    size = desc.DynamicNodeSize("input")
    category = "Utils"
    documentation = """
    Takes a sfmData as input.
    If the sfmData contained a rig, each view will be transformed such that they point to 
    individual independent poses. The absolute pose of each view is kept numerically.
    """

    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
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
            value="{nodeCacheFolder}/sfmData.abc",
        )
    ]

    def processChunk(self, chunk):

        from pyalicevision import sfmData as avsfmdata
        from pyalicevision import sfmDataIO as avsfmdataio

        import logging
        logging.getLogger().setLevel(chunk.node.verboseLevel.value.upper())

        # Open SfMData
        data = avsfmdata.SfMData()
        ret = avsfmdataio.load(data, chunk.node.input.value, avsfmdataio.ALL)
        if not ret:
            logging.error(f"Can't open sfmData file at {chunk.node.input.value}")
            raise RuntimeError()

        logging.info(f"Opened sfmData at {chunk.node.input.value}")

        views = data.getViews()
        poses = {}

        # Backup the absolute pose of each view using the rig
        for id, v in views.items():
            if data.isPoseDefined(v):
                poses[id] = data.getPose(v)
            
            # Remove all "rig" references
            v.setPoseId(id)
            v.setRigAndSubPoseId(avsfmdata.UndefinedIndexT, avsfmdata.UndefinedIndexT)

        data.getPoses().clear()
        data.getRigs().clear()

        # ReApply poses as independent poses
        for id, pose in poses.items():
            data.getPoses()[id] = pose
            
        # Save SfmData
        avsfmdataio.save(data, chunk.node.output.value, avsfmdataio.ALL)
        
