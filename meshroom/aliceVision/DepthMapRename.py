__version__ = "1.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import json
import os
from pathlib import Path
from shutil import copyfile


class DepthMapRename(desc.Node):

    size = desc.DynamicNodeSize("input")
    category = "Utils"
    documentation = """
Rename depth map from image name to new viewId
"""
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.File(
            name="depthMapFolder",
            label="Depth Map Folder",
            description="Folder containing depth maps to rename.",
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
            label="Result Folder",
            description="Folder containing renamed depth maps.",
            value="{nodeCacheFolder}",
        ),
    ]

    def processChunk(self, chunk):
        try:
            chunk.logManager.start(chunk.node.verboseLevel.value)
            chunk.logger.info("Start renaming")

            if not os.path.exists(chunk.node.output.value):
                os.mkdir(chunk.node.output.value)

            inputPath = Path(chunk.node.input.value)
            outputPath = Path(chunk.node.output.value)
            depthMapFolder = Path(chunk.node.depthMapFolder.value)

            if inputPath.suffix == ".abc":
                inputPath = inputPath.parent / "cameras.sfm"

            with open(inputPath, 'r', encoding='utf-8', errors='ignore') as f:
                data = json.load(f)

            for view in list(data["views"]):
                imageStem = Path(view["path"]).stem
                viewId = int(view["viewId"])
                depthMapPath = depthMapFolder / f"{imageStem}.exr"

                if depthMapPath.exists():
                    newName = f"{viewId}_depthMap.exr"
                    chunk.logger.debug(f"File {imageStem} rename with ID {viewId}.")
                    copyfile(depthMapPath, outputPath / newName)
                else:
                    chunk.logger.error(f"File {imageStem} not found among depth maps.")
            chunk.logger.info("End renaming")

        finally:
            chunk.logManager.end()
