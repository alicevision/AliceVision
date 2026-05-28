__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMPoseFlattening(desc.Node):
    """
Convert a rig-based SfMData into a flat set of independent camera poses.

If the input SfMData describes a multi-camera rig (where multiple views share a single
rig pose), this node transforms the representation so that each view has its own
independent absolute pose. The world-space position and orientation of each camera
are preserved numerically; only the rig hierarchy is removed. This is useful for
passing rig reconstructions to nodes that do not support rig structures.
"""


    size = desc.DynamicNodeSize("input")
    category = "Utils"
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
        
