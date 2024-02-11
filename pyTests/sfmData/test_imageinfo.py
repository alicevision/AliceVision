"""
Collection of unit tests for the ImageInfo class.
"""

from aliceVision import sfmData as av
from ..constants import IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA

##################
### List of functions:
# - ImageInfo(imagePath = "", width = 0, height = 0, metadata = map<string, string>()) => DONE
# - bool operator==(other) => DONE
# - string getImagePath() => DONE
# - size_t getWidth() => DONE
# - size_t getHeight() => DONE
# - pair<size_t, size_t> getImgSize() => DONE
# - ExposureSetting getCameraExposureSetting()
# - double getEv()
# - map<string, string>::const_iterator findMetadataIterator(name)
# - bool hasMetadata(vector<string> names) => DONE
# - bool hasGpsMetadata()
# - bool hasDigitMetadata(vector<string> names, bool isPositive) => DONE
# - double readRealNumber()
# - double getDoubleMetadata(vector<string> names) => DONE
# - double getDoubleMetadata(vector<string> names, double& val) => TODO: disable
# - double getIntMetadata(vector<string> names) => DONE
# - string getMetadataMake() => DONE
# - string getMetadataModel() => DONE
# - string getMetadataBodySerialNumber() => DONE
# - string getMetadataLensModel() => DONE
# - int getMetadataLensID() => DONE
# - string getMetadataLensSerialNumber() => DONE
# - double getMetadataFocalLength() => DONE
# - double getMetadataShutter() => DONE
# - double getMetadataFNumber() => DONE
# - double getMetadataISO() => DONE
# - EEXIFOrientation getMetadataOrientation() => DONE
# - Vec3 getGpsPositionFromMetadata() !!! not correctly binded (Vec3) !!!
# - void getGpsPositionWGS84FromMetadata(double& lat, double& lon, double& alt)
# - Vec3 getGpsPositionWGS84FromMetadata() !!! not correctly binded (Vec3) !!!
# - string getColorProfileFileName() => DONE
# - vector<int> getCameraMultiplicator()
# - bool getVignettingParams(vector<float> vignParam)
# - bool getChromaticAberrationParams(vector<float>& caGParam, vector<float>& caBParam,
#                                     vector<float>& caRParam)
# - bool hasMetadataDateTimeOriginal() => DONE
# - string getMetadataDateTimeOriginal() => DONE
# - int64_t getMetadataDateTimestamp() => DONE
# - double getSensorWidth()
# - double getSensorHeight()
# - map<string, string> getMetadata()
# - void setImagePath(string) => DONE
# - void setWidth(size_t) => DONE
# - void setHeight(size_t) => DONE
# - void setMetadata(map<string, string>) => DONE
# - void addMetadata(string key, string value) => DONE
# - void addDCPMetadata(imaage::DCPProfile& dcpProf)
# - void addVignettingMetadata(LensParam& lensParam)
# - void addChromaticMetadata(LensParam& lensParam)
# - void getSensorSize(vector<sensorDB::Datasheet> sensorDB, double& sensorWidth,
#                      double& sensorHeight, double& focalLengthmm,
#                      camera::EInitMode& intrinsicInitMode, bool verbose = false)
##################

def test_imageinfo_default_constructor():
    """ Test creating an ImageInfo object with default parameters and accessing its values. """
    image_info = av.ImageInfo()
    make = image_info.getMetadataMake()
    assert not make, "No metadata should be available: the camera make should not be retrieved"


def test_imageinfo_constructor():
    """ Test creating an ImageInfo with set values and accessing them. """
    image_info = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    make = image_info.getMetadataMake()
    assert make == "Apple", "Metadata should be available: the camera make should be retrieved"


def test_imageinfo_add_metadata():
    """ Test creating a default ImageInfo object and adding a custom metadata value to it. """
    image_info = av.ImageInfo()
    orientation = image_info.getMetadataOrientation()
    assert orientation == -1, "The 'Orientation' metadata has never been set and should return -1"

    image_info.addMetadata("Orientation", "6")
    orientation = image_info.getMetadataOrientation()
    assert orientation == 6, "The 'Orientation' metadata has not been correctly set"


def test_imageinfo_compare():
    """ Test creating two ImageInfo objects and comparing them using the '==' operator. """
    image_info1 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    image_info2 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    assert image_info1 == image_info2, "The two ImageInfo objects should be equal"

    # The '==' operator should only take the width and height into account when comparing
    image_info1.setImagePath("/data/img.jpg")
    assert image_info1.getImagePath() != image_info2.getImagePath(), \
        "The two ImageInfo objects have different image paths"
    assert image_info1 == image_info2, "The two ImageInfo objects should be equal: \
        only their image path differs"

    image_info1.setWidth(IMAGE_WIDTH + 1)
    assert image_info1 != image_info2, "The two ImageInfo objects should be different \
        as their width differs"


def test_imageinfo_set_size():
    """ Test creating an ImageInfo object and modifying its size. """
    image_info = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    assert image_info.getWidth() == IMAGE_WIDTH and image_info.getHeight() == IMAGE_HEIGHT
    assert image_info.getImgSize() == (image_info.getWidth(), image_info.getHeight())

    image_info.setWidth(IMAGE_WIDTH * 2)
    image_info.setHeight(IMAGE_HEIGHT * 2)
    assert image_info.getWidth() == IMAGE_WIDTH * 2
    assert image_info.getHeight() == IMAGE_HEIGHT * 2
    assert image_info.getImgSize() == (image_info.getWidth(), image_info.getHeight())


def test_imageinfo_has_metadata():
    """ Test creating initialized and uninitialized ImageInfo objects and checking whether
    they have some metadata. """
    image_info1 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    # Testing that a single metadata key exists
    assert image_info1.hasMetadata(["Software"]), "The initialized ImageInfo object \
        has a 'Software' metadata key"
    # Testing that at least one metadata key in a provided list exist
    assert image_info1.hasMetadata(["Software", "random"]), "The initialized ImageInfo \
        object has at least a 'Software' metadata key"
    # Testing that no provided metadata key exists
    assert not image_info1.hasMetadata(["random", "test"]), "The initialized ImageInfo \
        object has none of these metadata keys"

    # Testing that a metadata key has a digit value (that may be negative)
    aperture_value_key = "Exif:ApertureValue"
    assert image_info1.hasDigitMetadata([aperture_value_key], False), "The initialized \
        ImageInfo object has a digit value for the 'Exif:ApertureValue' metadata key"
    # Testing that a metadata key has a digit value (that is positive)
    assert image_info1.hasDigitMetadata([aperture_value_key], True), f"The initialized \
        ImageInfo object has a positive digit value for the {aperture_value_key} metadata key"
    # Testing that a metadata key that has a string value is not recognized as a digit value
    assert not image_info1.hasDigitMetadata(["Exif:LensMake"], False), "The initialized \
        ImageInfo object has a string value for the 'Exif:LensMake' metadata key"

    assert image_info1.hasMetadataDateTimeOriginal()

    image_info2 = av.ImageInfo()

    assert not image_info2.hasMetadata(["Software"]), \
        "The uninitialized ImageInfo object has no metadata"
    assert not image_info2.hasMetadata(["Software", "random"]), \
        "The uninitialized ImageInfo object has no metadata"


def test_imageinfo_get_metadata():
    """ Test creating an initialized ImageInfo object with known metadata and
    retrieving its metadata. """
    image_info = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    metadata = image_info.getMetadata()
    assert len(metadata) > 0

    assert image_info.getMetadataMake() == "Apple"
    assert image_info.getMetadataModel() == "iPhone 7"
    assert image_info.getMetadataLensModel() == "iPhone 7 back camera 3.99mm f/1.8"
    assert image_info.getMetadataLensID() == -1         # does not exist in the test metadata

    assert image_info.getMetadataFocalLength() == 3.99  # focal length
    assert image_info.getMetadataShutter() == 0.030303  # exposure time
    assert image_info.getMetadataFNumber() == 1.8       # fnumber
    assert image_info.getMetadataISO() == 40            # photographic sensitivity

    assert image_info.getMetadataBodySerialNumber() == ""  # does not exist in the test metadata
    assert image_info.getMetadataLensSerialNumber() == ""  # does not exist in the test metadata

    assert image_info.getMetadataDateTimeOriginal() == "2018:01:26 15:23:28"
    assert image_info.getMetadataDateTimestamp() == 64865114608

    assert image_info.getDoubleMetadata(["Exif:ApertureValue",
        "Exif:BrightnessValue"]) == 1.69599  # should retrieve first key value
    assert image_info.getIntMetadata(["Exif:PixelXDimension", "Exif:PixelYDimension"]) == 4032


def test_imageinfo_set_metadata():
    """ Test creating ImageInfo objects and overwriting their metadata. """
    image_info1 = av.ImageInfo()
    metadata = image_info1.getMetadata()
    assert len(metadata) == 0
    assert not image_info1.hasMetadata(["key_1", "key_2"])

    metadata = {"key_1": "value_1", "key_2": "value_2"}
    image_info1.setMetadata(metadata)
    assert image_info1.hasMetadata(["key_1", "key_2"])
    assert image_info1.getMetadata().keys() == list(metadata.keys())
    assert image_info1.getMetadata().values() == list(metadata.values())

    image_info2 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    assert image_info2.getMetadata().keys() == list(METADATA.keys())
    assert image_info2.getMetadata().values() == list(METADATA.values())

    image_info2.setMetadata(metadata)
    assert len(image_info2.getMetadata()) == len(metadata)
