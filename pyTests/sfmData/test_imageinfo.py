from ..utils import *
from ..constants import *

from aliceVision import sfmData as av

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
# - bool getChromaticAberrationParams(vector<float>& caGParam, vector<float>& caBParam, vector<float>& caRParam)
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
# - void getSensorSize(vector<sensorDB::Datasheet> sensorDB, double& sensorWidth, double& sensorHeight, double& focalLengthmm, camera::EInitMode& intrinsicInitMode, bool verbose = false)
##################


def test_imageinfo_default_constructor():
    """ Test creating an ImageInfo object with default parameters and accessing its values. """
    ii = av.ImageInfo()
    make = ii.getMetadataMake()
    assert not make, "No metadata should be available: the camera make should not be retrieved"


def test_imageinfo_constructor():
    """ Test creating an ImageInfo with set values and accessing them. """
    ii = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    make = ii.getMetadataMake()
    assert make == "Apple", "Metadata should be available: the camera make should be retrieved"


def test_imageinfo_add_metadata():
    """ Test creating a default ImageInfo object and adding a custom metadata value to it. """
    ii = av.ImageInfo()
    orientation = ii.getMetadataOrientation()
    assert orientation == -1, "The 'Orientation' metadata has never been set and should return -1"

    ii.addMetadata("Orientation", "6")
    orientation = ii.getMetadataOrientation()
    assert orientation == 6, "The 'Orientation' metadata has not been correctly set"


def test_imageinfo_compare():
    """ Test creating two ImageInfo objects and comparing them using the '==' operator. """
    ii1 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    ii2 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    assert ii1 == ii2, "The two ImageInfo objects should be equal"

    # The '==' operator should only take the width and height into account when comparing
    ii1.setImagePath("/tmp_path/data/img.jpg")
    assert ii1.getImagePath() != ii2.getImagePath(), "The two ImageInfo objects have different image paths"
    assert ii1 == ii2, "The two ImageInfo objects should be equal: only their image path differs"

    ii1.setWidth(IMAGE_WIDTH + 1)
    assert ii1 != ii2, "The two ImageInfo objects should be different as their width differs"


def test_imageinfo_set_size():
    """ Test creating an ImageInfo object and modifying its size. """
    ii = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    assert ii.getWidth() == IMAGE_WIDTH and ii.getHeight() == IMAGE_HEIGHT
    assert ii.getImgSize() == (ii.getWidth(), ii.getHeight())

    ii.setWidth(IMAGE_WIDTH * 2)
    ii.setHeight(IMAGE_HEIGHT * 2)
    assert ii.getWidth() == IMAGE_WIDTH * 2
    assert ii.getHeight() == IMAGE_HEIGHT * 2
    assert ii.getImgSize() == (ii.getWidth(), ii.getHeight())


def test_imageinfo_has_metadata():
    """ Test creating initialized and uninitialized ImageInfo objects and checking whether they have some metadata. """
    ii1 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    # Testing that a single metadata key exists
    assert ii1.hasMetadata(["Software"]), "The initialized ImageInfo object has a 'Software' metadata key"
    # Testing that at least one metadata key in a provided list exist
    assert ii1.hasMetadata(["Software", "random"]), "The initialized ImageInfo object has at least a 'Software' metadata key"
    # Testing that no provided metadata key exists
    assert not ii1.hasMetadata(["random", "test"]), "The initialized ImageInfo object has none of these metadata keys"

    # Testing that a metadata key has a digit value (that may be negative)
    assert ii1.hasDigitMetadata(["Exif:ApertureValue"], False), "The initialized ImageInfo object has a digit value for the 'Exif:ApertureValue' metadata key"
    # Testing that a metadata key has a digit value (that is positive)
    assert ii1.hasDigitMetadata(["Exif:ApertureValue"], True), "The initialized ImageInfo object has a positive digit value for the 'Exif:ApertureValue' metadata key"
    # Testing that a metadata key that has a string value is not recognized as a digit value
    assert not ii1.hasDigitMetadata(["Exif:LensMake"], False), "The initialized ImageInfo object has a string value for the 'Exif:LensMake' metadata key"

    assert ii1.hasMetadataDateTimeOriginal()

    ii2 = av.ImageInfo()

    assert not ii2.hasMetadata(["Software"]), "The uninitialized ImageInfo object has no metadata"
    assert not ii2.hasMetadata(["Software", "random"]), "The uninitialized ImageInfo object has no metadata"


def test_imageinfo_get_metadata():
    """ Test creating an initialized ImageInfo object with known metadata and retrieving its metadata. """
    ii = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)

    metadata = ii.getMetadata()
    assert len(metadata) > 0

    assert ii.getMetadataMake() == "Apple"
    assert ii.getMetadataModel() == "iPhone 7"
    assert ii.getMetadataLensModel() == "iPhone 7 back camera 3.99mm f/1.8"
    assert ii.getMetadataLensID() == -1         # does not exist in the test metadata

    assert ii.getMetadataFocalLength() == 3.99  # focal length
    assert ii.getMetadataShutter() == 0.030303  # exposure time
    assert ii.getMetadataFNumber() == 1.8       # fnumber
    assert ii.getMetadataISO() == 40            # photographic sensitivity

    assert ii.getMetadataBodySerialNumber() == ""  # does not exist in the test metadata
    assert ii.getMetadataLensSerialNumber() == ""  # does not exist in the test metadata

    assert ii.getMetadataDateTimeOriginal() == "2018:01:26 15:23:28"
    assert ii.getMetadataDateTimestamp() == 64865114608

    assert ii.getDoubleMetadata(["Exif:ApertureValue", "Exif:BrightnessValue"]) == 1.69599  # should retrieve first key value
    assert ii.getIntMetadata(["Exif:PixelXDimension", "Exif:PixelYDimension"]) == 4032


def test_imageinfo_set_metadata():
    """ Test creating ImageInfo objects and overwriting their metadata. """
    ii1 = av.ImageInfo()
    metadata = ii1.getMetadata()
    assert len(metadata) == 0
    assert not ii1.hasMetadata(["key_1", "key_2"])

    metadata = {"key_1": "value_1", "key_2": "value_2"}
    ii1.setMetadata(metadata)
    assert ii1.hasMetadata(["key_1", "key_2"])
    assert ii1.getMetadata().keys() == list(metadata.keys())
    assert ii1.getMetadata().values() == list(metadata.values())

    ii2 = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    assert ii2.getMetadata().keys() == list(METADATA.keys())
    assert ii2.getMetadata().values() == list(METADATA.values())

    ii2.setMetadata(metadata)
    assert len(ii2.getMetadata()) == len(metadata)


def test_imageinfo_get_color_profile():
    ii = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    assert ii.getColorProfileFileName() == ""

    ii.addMetadata("AliceVision:DCP:colorProfileFileName", "dcpFilename")
    assert ii.getColorProfileFileName() == "dcpFilename"


if __name__ == "__main__":
    print_yellow("~~~ Hello world! ~~~")
    ii = av.ImageInfo(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT, METADATA)
    vec = ii.getGpsPositionFromMetadata()
    print(vec)