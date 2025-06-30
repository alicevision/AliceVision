"""
Collection of unit tests for matching
"""

import pytest
import os

from pyalicevision import matching as m

def test_io():
    
    matches = m.IndMatches() 
    matches.append(m.IndMatch(0, 2))
    matches.append(m.IndMatch(3, 4))

    perdesc = m.MatchesPerDescType()
    perdesc[m.EImageDescriberType_SIFT] = matches

    final = m.PairwiseMatches()
    pair = m.Pair(0, 1)
    final[pair] = perdesc

    directory = os.path.abspath(os.path.dirname(__file__))
    assert m.Save(final, directory, "txt", False, ""), "Error saving"
    
    loaded = m.PairwiseMatches()

    views = m.IndexTSet()
    views.append(0)
    views.append(1)

    directories = m.StringVector()
    directories.append(directory)

    types = m.EImageDescriberTypeVector()
    types.append(m.EImageDescriberType_SIFT)

    assert m.Load(loaded, views, directories, types, 0, 0)
    assert len(loaded) == 1, "Invalid loaded content"