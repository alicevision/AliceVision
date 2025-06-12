"""
Collection of unit tests for image
"""

import pytest
import os

from pyalicevision import image as img
import numpy as np

def loop(image):

    array1 = image.getNumpyArray()

    #Write image to file    
    new_path = os.path.abspath(os.path.dirname(__file__)) + "/out.exr"
    optRead = img.ImageReadOptions(img.EImageColorSpace_NO_CONVERSION)
    optWrite = img.ImageWriteOptions()
    optWrite.toColorSpace(img.EImageColorSpace_NO_CONVERSION)
    img.writeImage(new_path, image, optWrite)
    
    #read it back from file
    other_image = image.__class__()
    img.readImage(new_path, other_image, optRead)
    array2 = other_image.getNumpyArray()

    assert np.array_equal(array1, array2), "images should be equal"

def test_default_constructor():

    src = np.random.randint(0, 255, size=(256, 256, 1), dtype='uint8')
    image = img.Image_uchar()
    image.fromNumpyArray(src)
    loop(image)

    image = img.Image_float()
    image.fromNumpyArray(np.float32(src))
    loop(image)

    src = np.random.randint(0, 255, size=(256, 256, 3), dtype='uint8')
    image = img.Image_RGBColor()
    image.fromNumpyArray(src)
    loop(image)

    image = img.Image_RGBfColor()
    image.fromNumpyArray(np.float32(src))
    loop(image)

    src = np.random.randint(0, 255, size=(256, 256, 4), dtype='uint8')
    image = img.Image_RGBAColor()
    image.fromNumpyArray(src)
    loop(image)
    
    image = img.Image_RGBAfColor()
    image.fromNumpyArray(np.float32(src))
    loop(image)
