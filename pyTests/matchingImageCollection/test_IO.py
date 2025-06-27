"""
Collection of unit tests for matchingImageCollection
"""

import pytest
import os

from pyalicevision import matchingImageCollection as mic

def test_IO():
    
    new_path = os.path.abspath(os.path.dirname(__file__)) + "/out.txt"

    matches = mic.PairSet()
    matches.insert(mic.Pair(0, 1))
    matches.insert(mic.Pair(2, 3))
    matches.insert(mic.Pair(0, 1))

    assert mic.savePairsToFile(new_path, matches), "Impossible to save to file"

    loaded = mic.PairSet()
    assert mic.loadPairsFromFile(new_path, loaded), "Impossible to load from file"

    assert(len(loaded) == 2)
    
    

