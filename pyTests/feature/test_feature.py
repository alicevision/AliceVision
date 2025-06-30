"""
Collection of unit tests for feature
"""

import pytest
import os

from pyalicevision import feature as feat

def test_io():
    
    regions = feat.FakeRegions()

    regions.Descriptors().append(feat.FakeDescriptor())
    regions.Features().append(feat.PointFeature())
    regions.Descriptors().append(feat.FakeDescriptor())
    regions.Features().append(feat.PointFeature(1, 2, 0.5, 2))

    ffeat = os.path.abspath(os.path.dirname(__file__)) + "/out.feat"
    fdesc = os.path.abspath(os.path.dirname(__file__)) + "/out.desc"
    
    regions.Save(ffeat, fdesc), "Error Saving"

    compare = feat.FakeRegions()
    compare.Load(ffeat, fdesc)

    assert compare.RegionCount() == regions.RegionCount(), "Did not get the same collection"
