"""
Collection of unit tests for image
"""

import pytest

import image as img
import numpy as np

def test_default_constructor():
    image = img.Image_uchar()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 1), "Incorrect depth"
    assert (image.channels() == 1), "Incorrect channels"

    image = img.Image_float()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 4), "Incorrect depth"
    assert (image.channels() == 1), "Incorrect channels"


    image = img.Image_RGBAfColor()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 16), "Incorrect depth"
    assert (image.channels() == 4), "Incorrect channels"

    image = img.Image_RGBAColor()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 4), "Incorrect depth"
    assert (image.channels() == 4), "Incorrect channels"

    image = img.Image_RGBfColor()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 12), "Incorrect depth"
    assert (image.channels() == 3), "Incorrect channels"

    image = img.Image_RGBColor()
    assert (image.width() == 0), "default width is 0"
    assert (image.height() == 0), "default height is 0"
    assert (image.depth() == 3), "Incorrect depth"
    assert (image.channels() == 3), "Incorrect channels"

def test_constructor():
    image = img.Image_uchar(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 1), "Incorrect depth"
    assert (image.channels() == 1), "Incorrect channels"

    image = img.Image_float(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 4), "Incorrect depth"
    assert (image.channels() == 1), "Incorrect channels"


    image = img.Image_RGBAfColor(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 16), "Incorrect depth"
    assert (image.channels() == 4), "Incorrect channels"

    image = img.Image_RGBAColor(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 4), "Incorrect depth"
    assert (image.channels() == 4), "Incorrect channels"

    image = img.Image_RGBfColor(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 12), "Incorrect depth"
    assert (image.channels() == 3), "Incorrect channels"

    image = img.Image_RGBColor(10, 12)
    assert (image.width() == 10), "default width is 10"
    assert (image.height() == 12), "default height is 12"
    assert (image.depth() == 3), "Incorrect depth"
    assert (image.channels() == 3), "Incorrect channels"

