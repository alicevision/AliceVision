"""
Collection of unit tests for the brackets detection.
"""

import pytest

import aliceVision as av

##################
### List of functions:
# - operator<<() !!! not binded as such, replaced by "print" !!!
# - bool estimateBracketsFromSfmData(vector<vector<shared_ptr<View>>>& groups,SfMData& sfmData,
#                                    size_t countBrackets)
# - int selectTargetViews(vector<vector<shared_ptr<View>>>& groups, SfMData& sfmData,
#                         size_t countBrackets)
##################

@pytest.mark.skip(reason="Specific data needed")
def test_brackets_estimate_brackets():
    """ Test estimating the brackets (groups of View objects) from an SfMData. """
    assert True


@pytest.mark.skip(reason="Specific data needed")
def test_brackets_select_views():
    """ Test selecting the target views from the brackets groups. """
    assert True
