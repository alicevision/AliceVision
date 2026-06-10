"""
Collection of unit tests for track
"""

import pytest
import os
import tempfile
import numpy as np

from pyalicevision import track as t


def test_track_item_default():
    """Test default construction of TrackItem."""
    item = t.TrackItem()
    assert item.featureId == 0
    assert item.scale == 0.0
    assert item.depth == 0.0


def test_track_item_fields():
    """Test setting fields on a TrackItem."""
    item = t.TrackItem()
    item.featureId = 42
    item.scale = 1.5
    item.depth = 3.0
    item.coords = np.array([100.0, 200.0])

    assert item.featureId == 42
    assert item.scale == pytest.approx(1.5)
    assert item.depth == pytest.approx(3.0)
    coords = np.array(item.coords)
    assert coords[0] == pytest.approx(100.0)
    assert coords[1] == pytest.approx(200.0)


def test_track_default():
    """Test default construction of Track."""
    track = t.Track()
    assert len(track.featPerView) == 0
    assert track.descType == t.EImageDescriberType_UNINITIALIZED


def test_track_desc_type():
    """Test setting the descriptor type on a Track."""
    track = t.Track()
    track.descType = t.EImageDescriberType_SIFT
    assert track.descType == t.EImageDescriberType_SIFT


def test_track_feat_per_view():
    """Test adding TrackItems to a Track via featPerView."""
    track = t.Track()
    track.descType = t.EImageDescriberType_SIFT

    item0 = t.TrackItem()
    item0.featureId = 10
    item0.scale = 1.0
    item0.depth = 0.5
    item0.coords = np.array([50.0, 75.0])

    item1 = t.TrackItem()
    item1.featureId = 20
    item1.scale = 2.0
    item1.depth = 1.0
    item1.coords = np.array([150.0, 175.0])

    track.featPerView[0] = item0
    track.featPerView[1] = item1

    assert len(track.featPerView) == 2
    assert track.featPerView[0].featureId == 10
    assert track.featPerView[1].featureId == 20


def test_tracks_map():
    """Test building a TracksMap with multiple tracks."""
    tracks = t.TracksMap()

    for track_id in range(3):
        track = t.Track()
        track.descType = t.EImageDescriberType_SIFT
        for view_id in range(2):
            item = t.TrackItem()
            item.featureId = track_id * 10 + view_id
            item.scale = float(track_id + 1)
            item.depth = float(view_id)
            item.coords = np.array([float(track_id * 100), float(view_id * 100)])
            track.featPerView[view_id] = item
        tracks[track_id] = track

    assert len(tracks) == 3
    assert tracks[0].featPerView[0].featureId == 0
    assert tracks[1].featPerView[1].featureId == 11
    assert tracks[2].featPerView[0].featureId == 20


def test_track_io():
    """Test saving and loading a TracksMap to/from a JSON file."""
    tracks = t.TracksMap()

    track = t.Track()
    track.descType = t.EImageDescriberType_SIFT

    item0 = t.TrackItem()
    item0.featureId = 5
    item0.scale = 1.2
    item0.depth = 2.5
    item0.coords = np.array([10.0, 20.0])
    track.featPerView[0] = item0

    item1 = t.TrackItem()
    item1.featureId = 7
    item1.scale = 0.8
    item1.depth = 1.0
    item1.coords = np.array([30.0, 40.0])
    track.featPerView[1] = item1

    tracks[0] = track

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        filepath = f.name

    try:
        assert t.saveTracks(tracks, filepath), "Error saving tracks"

        loaded = t.TracksMap()
        assert t.loadTracks(loaded, filepath), "Error loading tracks"

        assert len(loaded) == 1, "Expected 1 track after loading"
        loaded_track = loaded[0]
        assert len(loaded_track.featPerView) == 2

        assert loaded_track.featPerView[0].featureId == 5
        assert loaded_track.featPerView[0].scale == pytest.approx(1.2)
        assert loaded_track.featPerView[0].depth == pytest.approx(2.5)

        assert loaded_track.featPerView[1].featureId == 7
        assert loaded_track.featPerView[1].scale == pytest.approx(0.8)
        assert loaded_track.featPerView[1].depth == pytest.approx(1.0)
    finally:
        os.unlink(filepath)


def test_track_info_per_view():
    """Test TrackInfoPerView as a standalone map."""
    info = t.TrackInfoPerView()

    item = t.TrackItem()
    item.featureId = 99
    item.scale = 3.0
    item.depth = 7.0
    item.coords = np.array([1.0, 2.0])

    info[42] = item

    assert len(info) == 1
    assert 42 in info
    assert info[42].featureId == 99
