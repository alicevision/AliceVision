"""
Collection of unit tests for the rangeComputation utility function.
"""

import pytest

from pyalicevision import system as av

##################
### List of functions:
# - rangeComputation(rangeIteration, rangeBlocksCount, itemsCount) => DONE
#   Returns a tuple (bool, rangeStart, rangeEnd) via SWIG OUTPUT typemaps.
#   - rangeIteration: the current iteration number (0-based)
#   - rangeBlocksCount: total number of parallel blocks
#   - itemsCount: total number of items to process
#   - returns: (success, rangeStart, rangeEnd)
#     rangeEnd is non-inclusive
##################


def test_range_computation_single_block():
    """ Test rangeComputation with a single block covering all items. """
    result = av.rangeComputation(0, 1, 10)
    success, rangeStart, rangeEnd = result

    assert success is True, "Single block should process all items"
    assert rangeStart == 0, "Start should be 0"
    assert rangeEnd == 10, "End should be itemsCount"


def test_range_computation_two_blocks_first():
    """ Test rangeComputation with 2 blocks, requesting the first block. """
    result = av.rangeComputation(0, 2, 10)
    success, rangeStart, rangeEnd = result

    assert success is True
    assert rangeStart == 0
    assert rangeEnd == 5


def test_range_computation_two_blocks_second():
    """ Test rangeComputation with 2 blocks, requesting the second block. """
    result = av.rangeComputation(1, 2, 10)
    success, rangeStart, rangeEnd = result

    assert success is True
    assert rangeStart == 5
    assert rangeEnd == 10


def test_range_computation_covers_all_items():
    """ Test that all blocks together cover all items without gaps or overlaps. """
    items_count = 17
    blocks_count = 4
    all_indices = set()

    for iteration in range(blocks_count):
        result = av.rangeComputation(iteration, blocks_count, items_count)
        success, rangeStart, rangeEnd = result
        if success:
            for i in range(rangeStart, rangeEnd):
                assert i not in all_indices, \
                    f"Index {i} is covered by multiple blocks"
                all_indices.add(i)

    assert all_indices == set(range(items_count)), \
        "All items should be covered exactly once"


def test_range_computation_uneven_division():
    """ Test rangeComputation when items don't divide evenly into blocks. """
    items_count = 7
    blocks_count = 3
    total_items_covered = 0

    for iteration in range(blocks_count):
        result = av.rangeComputation(iteration, blocks_count, items_count)
        success, rangeStart, rangeEnd = result
        if success:
            assert rangeEnd > rangeStart, "Non-empty range expected"
            total_items_covered += (rangeEnd - rangeStart)

    assert total_items_covered == items_count, \
        "Total items covered should equal itemsCount"


def test_range_computation_more_blocks_than_items():
    """ Test rangeComputation when there are more blocks than items. """
    items_count = 3
    blocks_count = 10
    total_items_covered = 0

    for iteration in range(blocks_count):
        result = av.rangeComputation(iteration, blocks_count, items_count)
        success, rangeStart, rangeEnd = result
        if success:
            total_items_covered += (rangeEnd - rangeStart)

    assert total_items_covered == items_count, \
        "All items should still be covered even with excess blocks"


def test_range_computation_single_item():
    """ Test rangeComputation with a single item. """
    result = av.rangeComputation(0, 1, 1)
    success, rangeStart, rangeEnd = result

    assert success is True
    assert rangeStart == 0
    assert rangeEnd == 1


def test_range_computation_zero_items():
    """ Test rangeComputation with zero items returns false. """
    result = av.rangeComputation(0, 1, 0)
    success, _, _ = result
    assert success is False, "No items to process should return False"


def test_range_computation_out_of_range_iteration():
    """ Test rangeComputation with an iteration index beyond the block count. """
    result = av.rangeComputation(5, 3, 10)
    success, _, _ = result
    assert success is False, \
        "Iteration index beyond block count should return False"


def test_range_computation_large_dataset():
    """ Test rangeComputation with a large number of items. """
    items_count = 10000
    blocks_count = 7
    total_items_covered = 0

    for iteration in range(blocks_count):
        result = av.rangeComputation(iteration, blocks_count, items_count)
        success, rangeStart, rangeEnd = result
        if success:
            assert rangeStart >= 0
            assert rangeEnd <= items_count
            assert rangeEnd > rangeStart
            total_items_covered += (rangeEnd - rangeStart)

    assert total_items_covered == items_count
