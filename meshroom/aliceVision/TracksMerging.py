__version__ = "3.0"

from meshroom.core import desc
from meshroom.core.utils import DESCRIBER_TYPES, VERBOSE_LEVEL

import os.path


class TracksMerging(desc.Node):
    category = 'Utils'
    documentation = '''
Merges multiple track files into one
'''

    inputs = [
        desc.ListAttribute(
            elementDesc=desc.File(
                name="input",
                label="Input Track File",
                description="A track file.",
                value="",
            ),
            name="inputs",
            label="Inputs",
            description="Set of track files (at least 1 is required).",
            exposed=True,
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
            label="Output tracks file", 
            description="Path to the output track file",
            value="{nodeCacheFolder}/tracks.json",
        )
    ]

    def processChunk(self, chunk):
        from pyalicevision import track

        chunk.logManager.start(chunk.node.verboseLevel.value)

        trackOutput = track.TracksMap()

        #Loop over inputs
        pos = 0
        for input in chunk.node.inputs:
            
            chunk.logger.info(f"Processing input file {input.value}")
            trackInput = track.TracksMap()
            if not track.loadTracks(trackInput, input.value):
                chunk.logger.error("Cannot open input")
                chunk.logManager.end()
                raise RuntimeError()
            
            for key, value in trackInput.items():
                trackOutput[pos] = value
                pos = pos + 1

        chunk.logger.info(f"Save output to file {chunk.node.output.value}")
        track.saveTracks(trackOutput, chunk.node.output.value)

        chunk.logManager.end()