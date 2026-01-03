"""
Collection of unit tests for image
"""

import pytest
import os

from pyalicevision import image as img
import numpy as np

def loop(image, orientation : int = 1, pixelAspectRatio : float = 1.0, compression : str = "zips", name : str = "name", value : str = "value"):
    """
    Tests the integrity of an image I/O process by writing the image to disk, reading it back, and verifying that the output is identical to the original input.
    Raises:
        AssertionError: If the image or the metadata read from disk does not match the original inputs.
    """
    array1 = image.getNumpyArray()

    #Write image to file    
    new_path = os.path.abspath(os.path.dirname(__file__)) + "/out.exr"
    optRead = img.ImageReadOptions(img.EImageColorSpace_NO_CONVERSION)
    optWrite = img.ImageWriteOptions()
    optWrite.toColorSpace(img.EImageColorSpace_NO_CONVERSION)
    oiio_params = img.oiioParams(orientation, pixelAspectRatio, compression)
    oiio_params.add(name, value)

    img.writeImage(new_path, image, optWrite, oiio_params.get())

    #read it back from file
    other_image = image.__class__()
    img.readImage(new_path, other_image, optRead)
    array2 = other_image.getNumpyArray()

    metadata = img.readImageMetadataAsMap(new_path)
    pixelAspectRatio2 = float(metadata['PixelAspectRatio']) if 'PixelAspectRatio' in metadata else -1.0
    orientation2 = int(metadata['Orientation']) if 'Orientation' in metadata else -1
    compression2 = metadata['compression'] if 'compression' in metadata else "missing"
    value2 = metadata[name]

    info2 = [pixelAspectRatio2, orientation2, compression2, value2]
    # In case of existing 'compression' metadata but set as an empty string, oiio set 'zips' as default compression value
    info1 = [pixelAspectRatio, orientation, compression if len(compression)>0 else 'zips', value]

    os.remove(new_path)

    assert np.array_equal(array1, array2), "images should be equal"
    assert np.array_equal(info1, info2), "metadata should exist and be equal"

def test_default_constructor():

    src = np.random.randint(0, 255, size=(256, 256, 1), dtype='uint8')
    image = img.Image_uchar()
    image.fromNumpyArray(src)
    loop(image)

    image = img.Image_float()
    image.fromNumpyArray(np.float32(src))
    loop(image, orientation=2, pixelAspectRatio=1.33)

    src = np.random.randint(0, 255, size=(256, 256, 3), dtype='uint8')
    image = img.Image_RGBColor()
    image.fromNumpyArray(src)
    loop(image, orientation=3, pixelAspectRatio=2.0, compression="")

    src = np.random.randint(0, 255, size=(256, 256, 3), dtype='uint8')
    image = img.Image_RGBColor()
    image.fromNumpyArray(src)
    loop(image, orientation=3, pixelAspectRatio=2.0, compression="zips", name="metaName", value="metaValue")

    image = img.Image_RGBfColor()
    image.fromNumpyArray(np.float32(src))
    loop(image, orientation=4, pixelAspectRatio=1.0)

    src = np.random.randint(0, 255, size=(256, 256, 4), dtype='uint8')
    image = img.Image_RGBAColor()
    image.fromNumpyArray(src)
    loop(image, orientation=5, pixelAspectRatio=1.66)
    
    image = img.Image_RGBAfColor()
    image.fromNumpyArray(np.float32(src))
    loop(image, orientation=6)

