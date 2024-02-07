"""
Collection of constants that are used across different unit tests.
"""

import os

SFMDATA_PATH = os.path.abspath(os.path.dirname(__file__)) + "/data/small.sfm"

IMAGE_PATH = os.path.abspath(os.path.dirname(__file__)) + "/data/img.jpg"
IMAGE_WIDTH = 500
IMAGE_HEIGHT = 375

# Python dictionary containing the metadata of data/img.jpg
METADATA = {
    "AliceVision:SensorWidth": "4.890000",
    "DateTime": "2018:01:26 15:23:28",
    "Exif:ApertureValue": "1.69599",
    "Exif:BrightnessValue": "3.9586",
    "Exif:ColorSpace": "65535",
    "Exif:CustomRendered": "2",
    "Exif:DateTimeDigitized": "2018:01:26 15:23:28",
    "Exif:DateTimeOriginal": "2018:01:26 15:23:28",
    "Exif:ExifVersion": "0221",
    "Exif:ExposureBiasValue": "0",
    "Exif:ExposureMode": "0",
    "Exif:ExposureProgram": "2",
    "Exif:Flash": "16",
    "Exif:FlashPixVersion": "0100",
    "Exif:FocalLength": "3.99",
    "Exif:FocalLengthIn35mmFilm": "28",
    "Exif:LensMake": "Apple",
    "Exif:LensModel": "iPhone 7 back camera 3.99mm f/1.8",
    "Exif:LensSpecification": "3.99, 3.99, 1.8, 1.8",
    "Exif:MeteringMode": "5",
    "Exif:PhotographicSensitivity": "40",
    "Exif:PixelXDimension": "4032",
    "Exif:PixelYDimension": "3024",
    "Exif:SceneCaptureType": "0",
    "Exif:SensingMethod": "2",
    "Exif:ShutterSpeedValue": "5.05894",
    "Exif:SubsecTimeDigitized": "360",
    "Exif:SubsecTimeOriginal": "360",
    "Exif:WhiteBalance": "0",
    "Exif:YCbCrPositioning": "1",
    "ExposureTime": "0.030303",
    "FNumber": "1.8",
    "GPS:Altitude": "32.7506",
    "GPS:AltitudeRef": "0",
    "GPS:DateStamp": "2018:01:26",
    "GPS:DestBearing": "192.621",
    "GPS:DestBearingRef": "T",
    "GPS:HPositioningError": "65",
    "GPS:ImgDirection": "192.621",
    "GPS:ImgDirectionRef": "T",
    "GPS:Latitude": "48, 49, 55.46",
    "GPS:LatitudeRef": "N",
    "GPS:Longitude": "2, 16, 51.67",
    "GPS:LongitudeRef": "E",
    "GPS:Speed": "0",
    "GPS:SpeedRef": "K",
    "GPS:TimeStamp": "14, 23, 24",
    "ICCProfile": "0, 0, 2, 36, 97, 112, 112, 108, 4, 0, 0, 0, 109, 110, 116, \
        114, 82, 71, 66, 32, 88, 89, 90, 32, 7, 225, 0, 7, 0, 7, 0, 13, 0, 22, \
            0, 32, 97, 99, 115, 112, 65, 80, 80, 76, 0, 0, 0, 0, 65, 80, 80, 76, \
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ... [548 x uint8]",
    "ICCProfile:attributes": "Reflective, Glossy, Positive, Color",
    "ICCProfile:cmm_type": "1634758764",
    "ICCProfile:color_space": "RGB",
    "ICCProfile:copyright": "Copyright Apple Inc., 2017",
    "ICCProfile:creation_date": "2017:07:07 13:22:32",
    "ICCProfile:creator_signature": "6170706c",
    "ICCProfile:device_class": "Display device profile",
    "ICCProfile:flags": "Not Embedded, Independent",
    "ICCProfile:manufacturer": "4150504c",
    "ICCProfile:model": "0",
    "ICCProfile:platform_signature": "Apple Computer, Inc.",
    "ICCProfile:profile_connection_space": "XYZ",
    "ICCProfile:profile_description": "Display P3",
    "ICCProfile:profile_size": "548",
    "ICCProfile:profile_version": "4.0.0",
    "ICCProfile:rendering_intent": "Perceptual",
    "Make": "Apple",
    "Model": "iPhone 7",
    "Orientation": "1",
    "ResolutionUnit": "none",
    "Software": "11.2.2",
    "XResolution": "72",
    "YResolution": "72",
    "jpeg:subsampling": "4:2:0",
    "oiio:ColorSpace": "sRGB"
}

VIEW_ID = 12345
INTRINSIC_ID = 33
POSE_ID = 44
RIG_ID = 55
SUBPOSE_ID = 66
