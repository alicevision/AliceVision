__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


class ImagePairsMerging(desc.Node):

    category = "Utils"
    documentation = """Merge multiple image pairs files into a single one."""

    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input Image Pairs File",
                description="A file containing a list of image pairs.",
                value="",
            ),
            name="inputs",
            label="Inputs",
            description="Set of image pairs files (at least 1 is required).",
            exposed=True,
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
            label="Image Pairs",
            description="Filepath to the output file with the merged list of image pairs.",
            value="{nodeCacheFolder}/imageMatches.txt",
        ),
    ]

    def processChunk(self, chunk):
        from pyalicevision import matchingImageCollection as mic

        try:
            chunk.logManager.start(chunk.node.verboseLevel.value)

            inputFiles = [f.value for f in chunk.node.inputs.value if f.value]

            if not inputFiles:
                error = "No input image pairs files provided."
                chunk.logger.error(error)
                raise RuntimeError(error)

            mergedPairs = mic.PairSet()

            for inputFile in inputFiles:
                chunk.logger.info(f"Loading image pairs from: {inputFile}")
                sizeBefore = len(mergedPairs)
                if not mic.loadPairsFromFile(inputFile, mergedPairs):
                    error = f"Failed to load image pairs from file: {inputFile}"
                    chunk.logger.error(error)
                    raise RuntimeError(error)
                chunk.logger.info(f"Loaded {len(mergedPairs) - sizeBefore} pairs from {inputFile}")

            chunk.logger.info(f"Total merged pairs: {len(mergedPairs)}")
            chunk.logger.info(f"Saving merged image pairs to: {chunk.node.output.value}")

            if not mic.savePairsToFile(chunk.node.output.value, mergedPairs):
                error = f"Failed to save merged image pairs to: {chunk.node.output.value}"
                chunk.logger.error(error)
                raise RuntimeError(error)

            chunk.logger.info("Image pairs merging done.")

        finally:
            chunk.logManager.end()
