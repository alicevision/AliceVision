__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class SfMChecking(desc.Node):
    """
Validate an SfM scene against configurable quality constraints.

This node inspects the input SfMData and verifies that it meets minimum quality
requirements, such as a minimum percentage of reconstructed views. If the scene
does not satisfy the specified constraints, the node raises an error, which causes
the pipeline to stop. This is useful as a quality gate to prevent poor reconstructions
from propagating into computationally expensive downstream steps.
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
        desc.FloatParam(
            name="poseCompletion",
            label="Completion Percentage",
            description="Minimal percent of the views reconstructed.",
            value=80.0,
            range=(0.0, 100.0, 1.0),
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

        chunk.logManager.start(chunk.node.verboseLevel.value)
        chunk.logger.info("Open input file")

        data = avsfmdata.SfMData()
        ret = avsfmdataio.load(data, chunk.node.input.value, avsfmdataio.ALL)
        if not ret:
            chunk.logger.error("Cannot open input")
            chunk.logManager.end()
            raise RuntimeError()

        total = len(data.getViews())
        valid = len(data.getValidViews())
        ratio = (100.0 * float(valid)) / float(total)

        chunk.logger.info(f"Total views: {total}")
        chunk.logger.info(f"Reconstructed views: {valid}")
        chunk.logger.info(f"Percentage of reconstructed views: {ratio}")

        if ratio < chunk.node.poseCompletion.value:
            chunk.logger.error("Percentage of reconstructed views is insufficient.")
            chunk.logger.error(f"Expected {chunk.node.poseCompletion.value}, got {ratio}.")
            chunk.logManager.end()
            raise RuntimeError()

        avsfmdataio.save(data, chunk.node.output.value, avsfmdataio.ALL)

        chunk.logManager.end()
