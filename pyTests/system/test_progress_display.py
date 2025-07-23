"""
Collection of unit tests for the ProgressDisplay logger.
"""

import pytest

from pyalicevision.system import ConsoleProgressDisplay


PROGRESS_BAR_TEMPLATE = """{desc}
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
"""

def test_progress_display(capfd):
    items = range(10)
    desc = "Iter over items"
    stars_by_item = "*" * (50 // len(items))
    progress = ConsoleProgressDisplay(len(items), desc + "\n")
    for index, _i in enumerate(items):
        assert(progress.count() == index)
        out, _ = capfd.readouterr()  # readout is flushed each time we call this so we only have to check for diff
        if index == 0:
            assert out == PROGRESS_BAR_TEMPLATE.format(desc=desc)
        else:
            assert out == stars_by_item
        progress += 1
