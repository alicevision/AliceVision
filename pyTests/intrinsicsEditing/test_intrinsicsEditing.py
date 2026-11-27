"""
Collection of unit tests for the intrinsicsEditing module.
"""

import os

import numpy as np
import pytest

from pyalicevision import camera, intrinsicsEditing, sfmData, sfmDataIO

INTRINSIC_ID = 1
WIDTH = 4000
HEIGHT = 3000
SCALE = 3000.0

DEFAULTS = {
    "focalLength": 0.0,
    "pixelRatio": 0.0,
    "centerOffsetX": -100000.0,
    "centerOffsetY": -100000.0,
    "sensorWidth": 0.0,
    "sensorHeight": 0.0,
    "circleCenterX": -100000.0,
    "circleCenterY": -100000.0,
    "circleRadius": 0.0,
}


def _writeInput(path, sensorWidth, width=WIDTH, height=HEIGHT):
    """Save an SfMData with one pinhole intrinsic, used by one view (formats like USD only keep used intrinsics)."""
    data = sfmData.SfMData()
    intrinsic = camera.Pinhole(width, height, SCALE, SCALE, 0.0, 0.0)
    intrinsic.setSensorWidth(sensorWidth)
    intrinsic.setSensorHeight(24.0)
    data.getIntrinsics()[INTRINSIC_ID] = intrinsic
    data.getViews()[10] = sfmData.View("/tmp/img.jpg", 10, INTRINSIC_ID, 20, width, height)
    assert sfmDataIO.save(data, path, sfmDataIO.ALL)


def _edit(inputPath, outputPath, edits):
    """Run editSfmData with the fields filled from the input, then `edits` applied to them."""
    data = sfmData.SfMData()
    assert sfmDataIO.load(data, inputPath, sfmDataIO.ALL)
    _, values = intrinsicsEditing.readIntrinsic(data, intrinsicsEditing.AUTO)
    snapshot = {name: values.get(name, DEFAULTS[name]) for name in DEFAULTS}
    fields = dict(snapshot, **edits)
    intrinsicsEditing.editSfmData(inputPath, outputPath, intrinsicsEditing.AUTO, "pinhole", fields, snapshot,
                                  {}, True, DEFAULTS)


def _readScale(path):
    data = sfmData.SfMData()
    assert sfmDataIO.load(data, path, sfmDataIO.ALL)
    intrinsic = camera.IntrinsicScaleOffset.cast(data.getIntrinsics()[INTRINSIC_ID])
    return np.ravel(intrinsic.getScale()).tolist()


@pytest.mark.parametrize("sensorWidth", [0.0, -10.0])
def test_focal_length_edit_rejects_invalid_input_sensor_width(tmp_path, sensorWidth):
    """ A focal length edit with an input sensor width <= 0 is rejected instead of writing an invalid scale. """
    inputPath, outputPath = str(tmp_path / "in.usda"), str(tmp_path / "out.usda")
    _writeInput(inputPath, sensorWidth)

    with pytest.raises(ValueError, match="sensor width"):
        _edit(inputPath, outputPath, {"focalLength": 50.0})
    assert not os.path.exists(outputPath)


def test_focal_length_edit_with_sensor_width_edit(tmp_path):
    """ Editing the sensor width along with the focal length fixes an input sensor width <= 0. """
    inputPath, outputPath = str(tmp_path / "in.usda"), str(tmp_path / "out.usda")
    _writeInput(inputPath, 0.0)

    _edit(inputPath, outputPath, {"focalLength": 50.0, "sensorWidth": 36.0})
    expected = 50.0 * WIDTH / 36.0
    assert _readScale(outputPath) == pytest.approx([expected, expected])


def test_focal_length_edit_rejects_invalid_scale(tmp_path):
    """ A focal length edit giving a non-finite scale (image size of 0) is rejected. """
    inputPath, outputPath = str(tmp_path / "in.usda"), str(tmp_path / "out.usda")
    _writeInput(inputPath, 36.0, width=0, height=0)

    with pytest.raises(ValueError, match="Invalid scale"):
        _edit(inputPath, outputPath, {"focalLength": 50.0})
    assert not os.path.exists(outputPath)
