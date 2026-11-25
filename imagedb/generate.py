#!/usr/bin/env python3
"""Generate the imagedb dataset used to exercise every code path of aliceVision_cameraInit.

Dev-only script: its output (data/ and cases.json) is committed, so the tests never need to run it.
JPEGs need Pillow:
    rez env pillow-12.1.1 -- python3 imagedb/generate.py
DNG, DCP, LCP, sensor database and .sfm files only use the standard library (see tifflib.py).

Every case of cases.json lists the branch ids it covers; README.md maps those ids to the code.
"""
import json
import logging
import shutil
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tifflib import (ASCII, BYTE, LONG, RATIONAL, SHORT, SRATIONAL, write_tiff)  # noqa: E402

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
SFM_VERSION = ["1", "2", "14"]

logger = logging.getLogger("imagedb")


# ---------------------------------------------------------------------------------------------------------------------
# JPEG
# ---------------------------------------------------------------------------------------------------------------------

EXIF_IFD = 0x8769
TAGS_IFD0 = {"Make": 0x010F, "Model": 0x0110, "DateTime": 0x0132}
TAGS_EXIF = {
    "FocalLength": 0x920A, "FocalLengthIn35mmFilm": 0xA405, "BodySerialNumber": 0xA431, "LensSerialNumber": 0xA435,
    "LensModel": 0xA434, "FNumber": 0x829D, "ApertureValue": 0x9202, "PixelXDimension": 0xA002,
    "PixelYDimension": 0xA003, "DateTimeOriginal": 0x9003, "SubsecTimeOriginal": 0x9291, "ImageUniqueID": 0xA420,
}
RATIONAL_TAGS = {"FocalLength", "FNumber", "ApertureValue"}


def jpeg(path, size=(16, 12), color=(120, 90, 60), **exif):
    """Write a tiny JPEG with the given EXIF fields (names of TAGS_IFD0 / TAGS_EXIF)."""
    from PIL import Image
    from PIL.TiffImagePlugin import IFDRational

    path.parent.mkdir(parents=True, exist_ok=True)
    im = Image.new("RGB", size, color)
    ex = Image.Exif()
    sub = ex.get_ifd(EXIF_IFD)
    for key, value in exif.items():
        if key in RATIONAL_TAGS:
            value = IFDRational(value).limit_rational(100000)
            value = IFDRational(*value)
        if key in TAGS_IFD0:
            ex[TAGS_IFD0[key]] = value
        elif key in TAGS_EXIF:
            sub[TAGS_EXIF[key]] = value
        else:
            raise KeyError(key)
    im.save(path, exif=ex, quality=40)


# ---------------------------------------------------------------------------------------------------------------------
# DNG (32x24: LibRaw rejects images smaller than 22 pixels)
# ---------------------------------------------------------------------------------------------------------------------

COLOR_MATRIX = [0.8, -0.2, -0.1, -0.3, 1.1, 0.2, -0.05, 0.15, 0.6]
FORWARD_MATRIX = [0.7, 0.2, 0.1, 0.3, 0.6, 0.1, 0.0, 0.1, 0.8]


def dng(path, make, model, focal=None):
    width, height = 32, 24
    pixels = b"".join(struct.pack("<H", 1000 + ((x + y) % 4) * 500) for y in range(height) for x in range(width))
    tags = [
        (254, LONG, 0), (256, LONG, width), (257, LONG, height), (258, SHORT, 16), (259, SHORT, 1),
        (262, SHORT, 32803), (273, LONG, 0), (274, SHORT, 1), (277, SHORT, 1), (278, LONG, height),
        (279, LONG, len(pixels)), (284, SHORT, 1),
        (33421, SHORT, [2, 2]), (33422, BYTE, [0, 1, 1, 2]),
        (50706, BYTE, [1, 4, 0, 0]), (50707, BYTE, [1, 1, 0, 0]), (50708, ASCII, f"{make} {model}"),
        (50714, LONG, 0), (50717, LONG, 4095), (50721, SRATIONAL, COLOR_MATRIX),
        (50727, RATIONAL, [1.0, 1.0, 1.0]), (50728, RATIONAL, [0.5, 1.0, 0.7]), (50778, SHORT, 21),
    ]
    if make:
        tags.append((271, ASCII, make))
    if model:
        tags.append((272, ASCII, model))
    if focal:
        tags.append((37386, RATIONAL, focal))
    path.parent.mkdir(parents=True, exist_ok=True)
    write_tiff(path, tags, pixel_data=pixels)


# ---------------------------------------------------------------------------------------------------------------------
# DCP (TIFF-like, magic "IIRC")
# ---------------------------------------------------------------------------------------------------------------------

def dcp(path, illuminants=(17, 21), forward=True, camera_calibration=False, color_matrix=True, raw_bytes=None,
        extra_tags=()):
    path.parent.mkdir(parents=True, exist_ok=True)
    if raw_bytes is not None:
        path.write_bytes(raw_bytes)
        return
    tags = [(50708, ASCII, path.stem), (50932, ASCII, "com.adobe"), (50936, ASCII, "imagedb")] + list(extra_tags)
    if illuminants:
        tags.append((50778, SHORT, illuminants[0]))
        if len(illuminants) > 1:
            tags.append((50779, SHORT, illuminants[1]))
    if color_matrix:
        tags.append((50721, SRATIONAL, COLOR_MATRIX))
        if len(illuminants) > 1:
            tags.append((50722, SRATIONAL, COLOR_MATRIX))
    if forward:
        tags.append((50964, SRATIONAL, FORWARD_MATRIX))
        if len(illuminants) > 1:
            tags.append((50965, SRATIONAL, FORWARD_MATRIX))
    if camera_calibration:
        identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        tags.append((50723, SRATIONAL, identity))
        if len(illuminants) > 1:
            tags.append((50724, SRATIONAL, identity))
    write_tiff(path, tags, magic=0x4352)


# ---------------------------------------------------------------------------------------------------------------------
# LCP (Adobe lens correction profile, XMP)
# ---------------------------------------------------------------------------------------------------------------------

def _lcp_model(focal, aperture, fisheye, vignette, chromatic, geometry=True):
    geo = "FisheyeModel" if fisheye else "PerspectiveModel"
    center = ("<stCamera:ImageXCenter>0.51</stCamera:ImageXCenter>"
              "<stCamera:ImageYCenter>0.49</stCamera:ImageYCenter>")
    k3 = "" if fisheye else (f"<stCamera:RadialDistortParam3>{0.0003 + focal * 1e-6:.6f}</stCamera:RadialDistortParam3>"
                             "<stCamera:TangentialDistortParam1>0.0001</stCamera:TangentialDistortParam1>"
                             "<stCamera:TangentialDistortParam2>-0.0001</stCamera:TangentialDistortParam2>")
    inner = (f"<stCamera:Version>2</stCamera:Version><stCamera:FocalLengthX>1.2</stCamera:FocalLengthX>"
             f"<stCamera:FocalLengthY>1.2</stCamera:FocalLengthY>{center}"
             f"<stCamera:RadialDistortParam1>{0.01 + focal * 1e-4:.6f}</stCamera:RadialDistortParam1>"
             f"<stCamera:RadialDistortParam2>-0.002</stCamera:RadialDistortParam2>{k3}")
    if vignette:
        inner += ("<stCamera:VignetteModel rdf:parseType=\"Resource\">"
                  f"<stCamera:FocalLengthX>1.1</stCamera:FocalLengthX><stCamera:FocalLengthY>1.1</stCamera:FocalLengthY>{center}"
                  f"<stCamera:VignetteModelParam1>{-0.3 - aperture * 0.01:.4f}</stCamera:VignetteModelParam1>"
                  "<stCamera:VignetteModelParam2>0.05</stCamera:VignetteModelParam2>"
                  "<stCamera:VignetteModelParam3>-0.01</stCamera:VignetteModelParam3></stCamera:VignetteModel>")
    if chromatic:
        for name in ("ChromaticGreenModel", "ChromaticRedGreenModel", "ChromaticBlueGreenModel"):
            inner += (f"<stCamera:{name} rdf:parseType=\"Resource\">"
                      f"<stCamera:FocalLengthX>1.1</stCamera:FocalLengthX><stCamera:FocalLengthY>1.1</stCamera:FocalLengthY>{center}"
                      "<stCamera:RadialDistortParam1>0.001</stCamera:RadialDistortParam1>"
                      f"<stCamera:ScaleFactor>1.0002</stCamera:ScaleFactor></stCamera:{name}>")
    settings = (f"<stCamera:FocalLength>{focal}</stCamera:FocalLength><stCamera:FocusDistance>0</stCamera:FocusDistance>"
                f"<stCamera:ApertureValue>{aperture}</stCamera:ApertureValue>")
    if not geometry:
        return settings
    return settings + f"<stCamera:{geo} rdf:parseType=\"Resource\">{inner}</stCamera:{geo}>"


def lcp(path, make="Canon", model="Canon EOS 5D", lens="EF50mm f/1.8", lens_pretty="Canon EF 50mm f/1.8",
        camera_pretty=None,
        lens_ids=(-1,), raw_profile=True, fisheye=False, vignette=True, chromatic=True,
        focals=(24, 70), apertures=(3.0, 5.0), geometry=True, alternate_names=(), alternate_ids=(), broken=False,
        entries=None, fisheye_focals=()):
    """Write an LCP file (element style): one profile entry per (focal, aperture), or per item of `entries`.

    The first entry also carries the header. Focals listed in `fisheye_focals` get a FisheyeModel.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if broken:
        path.write_text("<x:xmpmeta><rdf:RDF><unclosed>\n")
        return
    header = (f"<stCamera:Author>imagedb</stCamera:Author><stCamera:ProfileName>{path.stem}</stCamera:ProfileName>"
              f"<stCamera:Make>{make}</stCamera:Make><stCamera:Model>{model}</stCamera:Model>"
              f"<stCamera:CameraPrettyName>{camera_pretty or model}</stCamera:CameraPrettyName>"
              f"<stCamera:Lens>{lens}</stCamera:Lens><stCamera:LensPrettyName>{lens_pretty}</stCamera:LensPrettyName>"
              + "".join(f"<stCamera:LensID>{i}</stCamera:LensID>" for i in lens_ids)
              + f"<stCamera:LensInfo>50/1 50/1 18/10 18/10</stCamera:LensInfo>"
              f"<stCamera:ImageWidth>4368</stCamera:ImageWidth><stCamera:ImageLength>2912</stCamera:ImageLength>"
              f"<stCamera:XResolution>72</stCamera:XResolution><stCamera:YResolution>72</stCamera:YResolution>"
              f"<stCamera:CameraRawProfile>{'True' if raw_profile else 'False'}</stCamera:CameraRawProfile>"
              f"<stCamera:SensorFormatFactor>1</stCamera:SensorFormatFactor>")
    if alternate_ids:
        header += ("<stCamera:AlternateLensIDs><rdf:Seq>" + "".join(f"<rdf:li>{i}</rdf:li>" for i in alternate_ids)
                   + "</rdf:Seq></stCamera:AlternateLensIDs>")
    if alternate_names:
        header += ("<stCamera:AlternateLensNames><rdf:Seq>" + "".join(f"<rdf:li>{n}</rdf:li>" for n in alternate_names)
                   + "</rdf:Seq></stCamera:AlternateLensNames>")
    pairs = entries if entries is not None else [(f, a) for a in apertures for f in focals]
    entries = [_lcp_model(f, a, fisheye or f in fisheye_focals, vignette, chromatic, geometry) for f, a in pairs]
    items = [f"<rdf:li rdf:parseType=\"Resource\">{header}{entries[0]}</rdf:li>"]
    items += [f"<rdf:li rdf:parseType=\"Resource\">{e}</rdf:li>" for e in entries[1:]]
    path.write_text(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<x:xmpmeta xmlns:x=\"adobe:ns:meta/\">\n"
        "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n"
        "<rdf:Description rdf:about=\"\" xmlns:photoshop=\"http://ns.adobe.com/photoshop/1.0/\" "
        "xmlns:stCamera=\"http://ns.adobe.com/photoshop/1.0/camera-profile\">\n"
        "<photoshop:CameraProfiles><rdf:Seq>\n" + "\n".join(items) + "\n</rdf:Seq></photoshop:CameraProfiles>\n"
        "</rdf:Description></rdf:RDF></x:xmpmeta>\n")


def lcp_attribute_style(path, make="Canon", model="Canon EOS 5D", lens="EF50mm f/1.8",
                        lens_pretty="Canon EF 50mm f/1.8", lens_id=-1, entries=((24, 3.0, False), (70, 3.0, False),
                                                                                 (100, 3.0, True)),
                        alternate_names=(), alternate_ids=()):
    """LCP written with XML attributes, laid out like the Adobe profiles: the first rdf:Description carries the
    header and the first camera settings, each entry lists its alternate lens ids / names before its model."""
    def alternates():
        out = ""
        if alternate_ids:
            out += ("<stCamera:AlternateLensIDs><rdf:Seq>" + "".join(f"<rdf:li>{i}</rdf:li>" for i in alternate_ids)
                    + "</rdf:Seq></stCamera:AlternateLensIDs>\n")
        if alternate_names:
            out += ("<stCamera:AlternateLensNames><rdf:Seq>"
                    + "".join(f"<rdf:li>{n}</rdf:li>" for n in alternate_names)
                    + "</rdf:Seq></stCamera:AlternateLensNames>\n")
        return out

    def entry(focal, aperture, fisheye, nested=False, header=""):
        """nested: each model's values sit in a child rdf:Description (the other Adobe layout)."""
        geo = "FisheyeModel" if fisheye else "PerspectiveModel"
        # "TangentiallDistortParam" (sic) is the spelling lcp.cpp looks for
        k3 = "" if fisheye else (' stCamera:RadialDistortParam3="0.0004" stCamera:TangentiallDistortParam1="0.0002"'
                                 ' stCamera:TangentiallDistortParam2="-0.0002"')
        geo_attrs = (f'stCamera:Version="2" stCamera:FocalLengthX="1.2" stCamera:FocalLengthY="1.2" '
                     f'stCamera:ImageXCenter="0.5" stCamera:ImageYCenter="0.5" '
                     f'stCamera:RadialDistortParam1="{0.02 + focal * 1e-4:.5f}" stCamera:RadialDistortParam2="-0.003"{k3}')
        vign_attrs = ('stCamera:FocalLengthX="1.1" stCamera:FocalLengthY="1.1" stCamera:VignetteModelParam1="-0.4" '
                      'stCamera:VignetteModelParam2="0.06" stCamera:VignetteModelParam3="-0.02"')
        ca_attrs = 'stCamera:FocalLengthX="1.1" stCamera:FocalLengthY="1.1" stCamera:RadialDistortParam1="0.002" ' \
                   'stCamera:ScaleFactor="1.0003"'
        names = ("ChromaticGreenModel", "ChromaticRedGreenModel", "ChromaticBlueGreenModel")
        if nested:
            models = (f'<stCamera:{geo}><rdf:Description {geo_attrs}>'
                      f'<stCamera:VignetteModel><rdf:Description {vign_attrs}/></stCamera:VignetteModel>'
                      + "".join(f'<stCamera:{n}><rdf:Description {ca_attrs}/></stCamera:{n}>' for n in names)
                      + f'</rdf:Description></stCamera:{geo}>')
        else:
            models = (f'<stCamera:{geo} {geo_attrs}><stCamera:VignetteModel {vign_attrs}/>'
                      + "".join(f'<stCamera:{n} {ca_attrs}/>' for n in names) + f"</stCamera:{geo}>")
        settings = f'stCamera:FocalLength="{focal}" stCamera:FocusDistance="0" stCamera:ApertureValue="{aperture}"'
        return f"<rdf:li><rdf:Description {header}{settings}>\n{alternates()}{models}\n</rdf:Description></rdf:li>\n"

    header = (f'stCamera:Author="imagedb" stCamera:Make="{make}" stCamera:Model="{model}" '
              f'stCamera:CameraPrettyName="{model}" stCamera:Lens="{lens}" stCamera:LensPrettyName="{lens_pretty}" '
              f'stCamera:LensID="{lens_id}" stCamera:CameraRawProfile="True" stCamera:SensorFormatFactor="1" ')
    body = entry(*entries[0], header=header) + "".join(entry(*e) for e in entries[1:])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<x:xmpmeta xmlns:x=\"adobe:ns:meta/\">\n"
        "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n"
        "<rdf:Description rdf:about=\"\" xmlns:photoshop=\"http://ns.adobe.com/photoshop/1.0/\" "
        "xmlns:stCamera=\"http://ns.adobe.com/photoshop/1.0/camera-profile\">\n"
        "<photoshop:CameraProfiles><rdf:Seq>\n" + body +
        "</rdf:Seq></photoshop:CameraProfiles>\n</rdf:Description></rdf:RDF></x:xmpmeta>\n")


# ---------------------------------------------------------------------------------------------------------------------
# .sfm inputs
# ---------------------------------------------------------------------------------------------------------------------

INTRINSIC_TEMPLATE = {
    "width": "16", "height": "12", "sensorWidth": "36", "sensorHeight": "27", "serialNumber": "sfm",
    "type": "pinhole", "initializationMode": "estimated", "initialFocalLength": "50", "focalLength": "50",
    "pixelRatio": "1", "pixelRatioLocked": "true", "offsetLocked": "false", "scaleLocked": "false",
    "principalPoint": ["0", "0"], "distortionLocked": "false", "distortionInitializationMode": "none",
    "distortionParams": ["0", "0", "0"], "undistortionOffset": ["0", "0"], "undistortionParams": "",
    "distortionType": "radialk3", "undistortionType": "none", "locked": "false",
}


def view(view_id, metadata, path="img/base.jpg", **extra):
    """One sfm view. Paths are written as {DATA}/..., replaced by the runner."""
    v = {"viewId": str(view_id), "path": "{DATA}/" + path, "metadata": metadata}
    v.update({k: str(x) for k, x in extra.items()})
    return v


def sfm(name, views, intrinsics=(), version=SFM_VERSION, raw=None):
    path = DATA / "sfm" / f"{name}.sfm"
    path.parent.mkdir(parents=True, exist_ok=True)
    if raw is not None:
        path.write_text(raw)
        return f"sfm/{name}.sfm"
    content = {"version": list(version), "views": list(views)}
    if intrinsics:
        content["intrinsics"] = list(intrinsics)
    path.write_text(json.dumps(content, indent=2) + "\n")
    return f"sfm/{name}.sfm"


# ---------------------------------------------------------------------------------------------------------------------
# Cases
# ---------------------------------------------------------------------------------------------------------------------

CASES = []


def case(name, covers, args=(), folder=None, sfm_file=None, expect="ok", log=(), sensor_db="sensors.db",
         env=None, note="", platform=None, default_output=True):
    """Declare a case. Inputs are relative to data/; "{DATA}" and "{OUT}" in args are replaced by the runner.

    expect: "ok" (exit 0, output compared), "fail" (non-zero exit, no output compared), "abort" (killed by a signal,
    e.g. std::terminate from an exception thrown inside an OpenMP region), "fail-or-abort".
    A "fail" or "abort" case must list a log message identifying the expected error.
    default_output: the runner adds "--output" when the arguments have none (False: arguments passed as they are).
    """
    assert expect == "ok" or log, f"{name}: a failing case needs a log message"
    env = dict(env or {})
    if expect == "abort":
        # with several threads throwing at once, std::terminate may not print the exception message
        env.setdefault("OMP_NUM_THREADS", "1")
    a = []
    if folder is not None:
        a += ["--imageFolder", "{DATA}/" + folder]
    if sfm_file is not None:
        a += ["--input", "{DATA}/" + sfm_file]
    a += list(args)
    CASES.append({"name": name, "covers": list(covers), "args": a, "expect": expect, "log": list(log),
                  "sensorDb": sensor_db, "env": env, "note": note, "platform": platform,
                  "defaultOutput": default_output})


def copy(src, dst):
    dst = DATA / dst
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(DATA / src, dst)


def build_images():
    img = DATA / "img"
    jpeg(img / "base.jpg")
    jpeg(img / "portrait.jpg", size=(12, 16))
    jpeg(img / "canon50.jpg", Make="Canon", Model="Canon EOS 5D", FocalLength=50, BodySerialNumber="1001",
         LensSerialNumber="2002", LensModel="EF50mm f/1.8", FNumber=1.8, PixelXDimension=16, PixelYDimension=12,
         DateTimeOriginal="2024:01:02 03:04:05", SubsecTimeOriginal="12", ImageUniqueID="uid-canon50")
    (img / "notes.txt").write_text("not an image\n")


def build_exif_folder():
    """Real JPEGs whose metadata is read by OIIO (view creation from files)."""
    f = DATA / "folders" / "exif"
    copy("img/canon50.jpg", "folders/exif/canon50.jpg")
    jpeg(f / "canon_unsure.jpg", Make="Canon", Model="5D", FocalLength=24, DateTimeOriginal="2024:01:02 03:04:06")
    jpeg(f / "canon_makemodel.jpg", Make="Canon", Model="EOS 5D", FocalLength=35)
    jpeg(f / "unknown_sensor.jpg", Make="Foo", Model="Bar", FocalLength=10)
    jpeg(f / "nometa.jpg")
    jpeg(f / "resized.jpg", Make="Testcam", Model="Model One", FocalLength=20, PixelXDimension=32,
         PixelYDimension=24)
    jpeg(f / "rotated.jpg", Make="Testcam", Model="Model One", FocalLength=20, PixelXDimension=12,
         PixelYDimension=16)
    jpeg(f / "zero_dims.jpg", Make="Testcam", Model="Model One", FocalLength=20, PixelXDimension=0,
         PixelYDimension=0)
    jpeg(f / "sentinel.jpg", Make="Testcam", Model="Model One", FocalLength=65535)
    jpeg(f / "only35.jpg", FocalLengthIn35mmFilm=28)
    jpeg(f / "focal_and_35.jpg", FocalLength=4.25, FocalLengthIn35mmFilm=26)
    jpeg(f / "db_and_35.jpg", Make="Canon", Model="Canon EOS 5D", FocalLengthIn35mmFilm=80)
    jpeg(f / "db_focal_and_35.jpg", Make="Canon", Model="Canon EOS 5D", FocalLength=50, FocalLengthIn35mmFilm=80)
    jpeg(f / "gopro_fisheye.jpg", Make="GoPro", Model="GoPro HERO9", FocalLength=0.4, BodySerialNumber="G9")
    jpeg(f / "custom.jpg", Make="Custom", Model="Rig", FocalLength=12)
    jpeg(f / "lensserial.jpg", Make="Testcam", Model="Model One", FocalLength=20, LensSerialNumber="L77")
    jpeg(f / "aperture_value.jpg", Make="Testcam", Model="Model One", FocalLength=20, ApertureValue=2.0)
    jpeg(f / "portrait_canon.jpg", size=(12, 16), Make="Canon", Model="Canon EOS 5D", FocalLength=50)


def build_listing_folders():
    copy("img/base.jpg", "folders/nested/a.jpg")
    copy("img/base.jpg", "folders/nested/sub/b.jpg")
    copy("img/base.jpg", "folders/nested/sub/deeper/c.jpg")
    copy("img/notes.txt", "folders/nested/notes.txt")
    copy("img/notes.txt", "folders/textonly/notes.txt")
    copy("img/base.jpg", "folders/viewid/img_0007.jpg")
    copy("img/base.jpg", "folders/viewid/img_0012.jpg")
    copy("img/base.jpg", "folders/viewid_nonumber/abc.jpg")
    # same stem, extension and folder-less metadata in two folders: distinct view ids thanks to the folder hash
    copy("img/base.jpg", "folders/uid/f1/same.jpg")
    copy("img/base.jpg", "folders/uid/f2/same.jpg")
    copy("img/canon50.jpg", "folders/uid/f1/withuid.jpg")
    jpeg(DATA / "folders/uid/f2/serial_only.jpg", BodySerialNumber="777", DateTime="2024:05:06 07:08:09")
    jpeg(DATA / "folders/uid/f2/date_only.jpg", DateTimeOriginal="2024:05:06 07:08:10")


def build_rig_folders():
    r = "folders/rig"
    for sub in ("0", "1"):
        for frame in ("0001", "0002"):
            copy("img/base.jpg", f"{r}/ok/rig/{sub}/frame_{frame}.jpg")
    copy("img/base.jpg", f"{r}/gap/rig/0/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/gap/rig/2/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/unequal/rig/0/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/unequal/rig/0/frame_0002.jpg")
    copy("img/base.jpg", f"{r}/unequal/rig/1/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/dupframe/rig/0/a_0001.jpg")
    copy("img/base.jpg", f"{r}/dupframe/rig/0/b_0001.jpg")
    copy("img/base.jpg", f"{r}/dupframe/rig/1/a_0001.jpg")
    copy("img/base.jpg", f"{r}/dupframe/rig/1/b_0001.jpg")
    copy("img/base.jpg", f"{r}/nonumber/rig/cam/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/nonumber/rig/cam/frame_0002.jpg")
    copy("img/base.jpg", f"{r}/overflow/rig/99999999999/frame_0001.jpg")
    copy("img/base.jpg", f"{r}/overflow/rig/99999999999/frame_0002.jpg")
    # frame ids parsed from file stems
    for stem in ("img_0012", "0013_left", "IMG_12_34", "abc", "n12345678901"):
        copy("img/base.jpg", f"folders/frames/{stem}.jpg")


def build_posegroup_folders():
    p = "folders/posegroups"
    for name in ("a", "ambient_light", "c"):
        copy("img/base.jpg", f"{p}/ps_one/{name}.jpg")
    # Without an ambient view, the views are sorted by (timestamp, view id): equal timestamps would leave the choice
    # of the middle view to the view ids, which hash absolute paths and file times. So every group below has an
    # ambient view or distinct timestamps.
    for name in ("AMBIENT", "y"):
        copy("img/base.jpg", f"{p}/PS_upper/{name}.jpg")
    for i, date in enumerate(("2024:01:01 10:00:02", "2024:01:01 10:00:00", "2024:01:01 10:00:01")):
        jpeg(DATA / f"{p}/hdr_dated/shot{i}.jpg", DateTimeOriginal=date)
    # one shot without date (timestamp -1) among dated ones
    copy("img/base.jpg", f"{p}/hdr_nodate/a.jpg")
    jpeg(DATA / f"{p}/hdr_nodate/b.jpg", DateTimeOriginal="2024:01:01 10:00:01")
    jpeg(DATA / f"{p}/hdr_nodate/c.jpg", DateTimeOriginal="2024:01:01 10:00:02")
    jpeg(DATA / f"{p}/HDR_upper/a.jpg", DateTimeOriginal="2024:01:01 11:00:01")
    jpeg(DATA / f"{p}/HDR_upper/b.jpg", DateTimeOriginal="2024:01:01 11:00:00")


def build_raw():
    dng(DATA / "raw/one/testcam.dng", "Testcam", "Model One", focal=20)
    dng(DATA / "raw/two/testcam_a.dng", "Testcam", "Model One", focal=20)
    dng(DATA / "raw/two/testcam_b.dng", "Testcam", "Model One", focal=20)
    dng(DATA / "raw/nomake/anon.dng", "", "", focal=20)
    dng(DATA / "raw/other/othercam.dng", "Othercam", "Model Two", focal=20)
    # DCP databases
    dcp(DATA / "dcp/full/Testcam Model One.dcp", illuminants=(17, 21), forward=True, camera_calibration=True)
    dcp(DATA / "dcp/single/Testcam Model One.dcp", illuminants=(21,), forward=False)
    dcp(DATA / "dcp/single/Othercam Model Two.dcp", illuminants=(21,), forward=True, camera_calibration=True)
    dcp(DATA / "dcp/nomatch/Somebody Else.dcp", illuminants=(21,))
    (DATA / "dcp/empty").mkdir(parents=True, exist_ok=True)
    (DATA / "dcp/empty/.keep").write_text("")
    dcp(DATA / "dcp/badtags/Testcam Model One.dcp", raw_bytes=b"IIRC" + struct.pack("<I", 8) + struct.pack("<H", 0))
    bad_type = (b"IIRC" + struct.pack("<I", 8) + struct.pack("<H", 1) + struct.pack("<HHI", 50721, 99, 9)
                + b"\0" * 8)
    dcp(DATA / "dcp/badtype/Testcam Model One.dcp", raw_bytes=bad_type)
    dcp(DATA / "dcp/nomatrix/Testcam Model One.dcp", illuminants=(21,), color_matrix=False, forward=False)
    build_rich_dcp()


def build_rich_dcp():
    """DCP profiles using the optional tags (look table, hue/sat maps, tone curves) and every illuminant value."""
    from tifflib import FLOAT, UNDEFINED
    hsv_dims = [2, 2, 1]
    hsv_data = [0.0, 1.0, 1.0] * 4                                  # hue shift, sat scale, val scale per entry
    rich = [
        (50981, LONG, hsv_dims), (50982, FLOAT, hsv_data), (51108, LONG, 1),          # look table + encoding
        (50937, LONG, hsv_dims), (50938, FLOAT, hsv_data), (50939, FLOAT, hsv_data),  # hue/sat maps 1 and 2
        (51107, LONG, 0), (50940, FLOAT, [0.0, 0.0, 0.5, 0.6, 1.0, 1.0]),            # encoding, non-linear curve
        (51109, SRATIONAL, -0.5), (51000, UNDEFINED, []),                            # baseline offset, empty tag
    ]
    profiles = {
        "Tone": ((24, 23), rich),                                                  # ISO studio tungsten, D50
        "Linear": ((20, 22), [(50940, FLOAT, [0.0, 0.0, 1.0, 1.0])]),             # D55, D75; linear curve
        "Copyright": ((12, 13), [(50942, ASCII, "Copyright Adobe Systems Inc.")]),  # fluorescent D, N
        "Fluo": ((14, 15), [(50942, ASCII, "Someone else")]),                     # fluorescent W, WW
        "Warm": ((16, 255), []),                                                   # fluorescent L, other
    }
    for model, (illuminants, extra) in profiles.items():
        dng(DATA / f"raw/rich/richcam_{model.lower()}.dng", "Richcam", model, focal=20)
        dcp(DATA / f"dcp/rich/Richcam {model}.dcp", illuminants=illuminants, extra_tags=extra)


def build_lcp():
    l = DATA / "lcp"
    lcp(l / "single/canon_ef50.lcp")
    lcp(l / "multi/canon_ef50_raw.lcp", lens_ids=(-1, 61))
    lcp(l / "multi/canon_ef50_jpeg.lcp", raw_profile=False, lens="EF50mm f/1.8 II")
    # no alternate lens names: element style with AlternateLensNames makes the full parse throw (see below)
    lcp(l / "multi/canon_fisheye.lcp", lens="EF8-15mm f/4L Fisheye", lens_pretty="Canon EF 8-15mm Fisheye",
        fisheye=True, chromatic=False)
    lcp(l / "multi/canon_vignetteonly.lcp", lens="EF35mm f/2", lens_pretty="Canon EF 35mm f/2", chromatic=False)
    lcp(l / "multi/nikon_50.lcp", make="Nikon", model="Nikon D800", lens="AF-S 50mm", lens_pretty="Nikkor 50mm")
    lcp(l / "multi/canon_upper.LCP", lens="IGNORED")
    # alternate lens names alone (AlternateLensIDs is never closed: names listed after ids are parsed as ids)
    lcp_attribute_style(l / "multi/canon_altnames.lcp", lens="EF100mm Macro", lens_pretty="Canon EF 100mm Macro",
                        entries=((10, 3.0, False), (70, 3.0, False)), alternate_names=("EF 100 Macro USM",))
    # alternate ids then names, attribute style (Adobe layout)
    lcp_attribute_style(l / "multi/canon_altboth.lcp", lens="EF85mm", lens_pretty="Canon EF 85mm",
                        entries=((10, 3.0, False), (70, 3.0, False)), alternate_ids=(-1, 300),
                        alternate_names=("EF 85 Alt",))
    # element style with alternate names: crashes the full parse (std::stoi), kept apart
    lcp(l / "altnames_element/canon_altnames_element.lcp", alternate_names=("EF 50 Alt",))
    # vignette apertures that decrease with the focal length: 5.0 at 24 mm, 3.0 at 70 mm
    lcp(l / "multi/canon_odd_aperture.lcp", lens="EF28-70 Odd", lens_pretty="Canon EF 28-70 Odd",
        entries=[(24, 5.0), (70, 3.0)])
    # fisheye entry at 24 mm and perspective entry at 70 mm: no geometry can be interpolated in between
    lcp(l / "multi/canon_mixed.lcp", lens="EF Mixed", lens_pretty="Canon EF Mixed", fisheye_focals=(24,),
        entries=[(24, 3.0), (70, 3.0)])
    # path contains "canon" but the profile is for another make: cameraOK is false
    lcp(l / "multi/canon_wrongmake.lcp", make="Kodak", model="Kodak X", lens="EF Wrong", lens_pretty="EF Wrong")
    # make and model differ from the image, but the camera pretty name equals the reduced make
    lcp(l / "multi/canon_prettyname.lcp", make="Canon Inc", model="EOS5D-internal", camera_pretty="Canon",
        lens="EF Pretty", lens_pretty="EF Pretty")
    lcp_attribute_style(l / "attr/canon_attr.lcp", entries=((24, 3.0, False), (70, 3.0, False, True),
                                                             (100, 3.0, True), (120, 3.0, True, True)))
    lcp(l / "nogeometry/canon_ef50_nogeo.lcp", geometry=False)
    lcp(l / "fisheye/canon_fisheye.lcp", fisheye=True, chromatic=False)
    lcp(l / "broken/canon_broken.lcp", broken=True)
    (l / "multi/readme.txt").write_text("not a profile\n")


def write_sensor_db():
    (DATA / "sensors.db").write_text(
        "# imagedb sensor database: brand;model;sensor width (mm);source\n"
        "\n"
        "Canon;Canon EOS 5D;35.8;imagedb\n"
        "GoPro;GoPro HERO9;6.17;imagedb\n"
        "Testcam;Model One;23.5;imagedb\n"
        "Othercam;Model Two;17.3;imagedb\n"
        "Negative;Width;-1;imagedb\n"
        "Negative;Two;-2;imagedb\n"
        "Short;Line;5\n")
    (DATA / "sensors_bad.db").write_text("Canon;Canon EOS 5D;not-a-number;imagedb\n")


# ---------------------------------------------------------------------------------------------------------------------
# Case list
# ---------------------------------------------------------------------------------------------------------------------

M_CANON = {"Make": "Canon", "Model": "Canon EOS 5D"}


def declare_cases():
    single = ["--allowSingleView", "1"]

    # --- 1. command line checks ------------------------------------------------------------------------------------
    case("args_none", ["A1"], expect="fail", sensor_db=None, default_output=False,
         log=["AliceVision cameraInit:"], note="no argument at all: CmdLine prints the options")
    case("args_unknown_option", ["A1"], args=["--imageFolder", "{DATA}/img", "--notAnOption", "1"], expect="fail",
         log=["unrecognised option '--notAnOption'"])
    case("args_bad_camera_model", ["A2"], folder="folders/exif", args=["--defaultCameraModel", "notamodel"],
         expect="fail", log=["[fatal] notamodel"])
    case("args_bad_distortion_model", ["A2"], folder="folders/exif",
         args=["--defaultCameraModel", "pinhole", "--defaultDistortionModel", "radialbrown"], expect="fail",
         log=["[fatal] radialbrown"],
         note="the help text advertises 'radialbrown', which is rejected")
    case("args_no_input", ["A3"], expect="fail", log=["Program need -i or --imageFolder option"])
    case("args_both_inputs", ["A4"], folder="folders/exif", sfm_file="sfm/sony.sfm", expect="fail",
         log=["Cannot combine -i and --imageFolder options"])
    case("args_missing_folder", ["A5"], args=["--imageFolder", "{DATA}/does_not_exist"], expect="fail",
         log=["The input folder doesn't exist"])
    case("args_missing_sfm", ["A6"], args=["--input", "{DATA}/does_not_exist.sfm"], expect="fail",
         log=["The input sfm file doesn't exist"])
    case("args_sfm_is_directory", ["A6"], args=["--input", "{DATA}/sfm"], expect="fail", log=["Is a directory"])
    case("args_empty_output", ["A7"], folder="folders/exif", args=["--output", ""], expect="fail",
         log=["Invalid output"])
    case("args_output_new_dir", ["A8"], folder="folders/exif", args=["--output", "{OUT}/newdir/cameraInit.sfm"],
         note="output parent created; the output is checked by the runner in newdir/")
    case("args_output_missing_parent", ["A8"], folder="folders/exif",
         args=["--output", "{OUT}/a/b/cameraInit.sfm"], expect="fail", log=["cannot create directory"])
    case("args_focal_and_fov", ["A9"], folder="folders/exif",
         args=["--defaultFocalLength", "10", "--defaultFieldOfView", "45"], expect="fail",
         log=["Cannot combine --defaultFocalLength --defaultFieldOfView options"])
    case("args_bad_focal_ratio", ["A10"], folder="folders/exif", args=["--defaultFocalRatio", "0"], expect="fail",
         log=["Focal Ratio needs to be a positive value"])
    case("args_no_sensor_db_no_root", ["A11"], folder="folders/exif", sensor_db=None,
         log=["ALICEVISION_SENSOR_DB is not defined and no sensor database is installed"])
    case("args_sensor_db_from_root", ["A11"], folder="folders/exif", sensor_db=None,
         env={"ALICEVISION_SENSOR_DB": "", "ALICEVISION_ROOT": "{DATA}/root"},
         note="an empty ALICEVISION_SENSOR_DB is ignored")
    case("args_sensor_db_from_env", ["A11"], folder="folders/exif", sensor_db=None,
         env={"ALICEVISION_SENSOR_DB": "{DATA}/sensors.db", "ALICEVISION_ROOT": "{DATA}/img"})
    case("args_sensor_db_missing", ["A12"], folder="folders/exif", sensor_db="does_not_exist.db", expect="fail",
         log=["Invalid input sensor database"])
    case("args_sensor_db_env_missing", ["A12"], folder="folders/exif", sensor_db=None,
         env={"ALICEVISION_SENSOR_DB": "{DATA}/does_not_exist.db", "ALICEVISION_ROOT": "{DATA}/root"},
         expect="fail", log=["Invalid input sensor database"],
         note="ALICEVISION_SENSOR_DB has priority over the database installed in ALICEVISION_ROOT")
    case("args_root_without_db", ["A12"], folder="folders/exif", sensor_db=None,
         env={"ALICEVISION_ROOT": "{DATA}/img"}, log=["no sensor database is installed in ALICEVISION_ROOT"])
    case("args_sensor_db_bad_width", ["A13"], folder="folders/exif", sensor_db="sensors_bad.db",
         expect="fail", log=["stod"])
    case("args_bad_output_extension", ["W4"], folder="folders/exif", args=["--output", "{OUT}/cameraInit.txt"],
         expect="fail", log=["The file extension is not recognized"])

    # --- 2. input listing and view creation --------------------------------------------------------------------------
    case("exif_folder", ["L7", "B3", "B4", "S1", "S2", "S3", "S4", "S5", "S7a", "S7b", "S8", "S10", "S11", "S12",
                         "S13", "FL1", "FL3", "G2", "G3", "G4", "G6", "G8", "G11", "G12", "G13", "N1", "N2", "N4",
                         "N5", "N8", "N9", "W1"],
         folder="folders/exif",
         log=["Resized image detected", "slightly different", "Sensor width doesn't exist",
              "no serial number"])
    case("exif_folder_fov", ["G1", "S12"], folder="folders/exif", args=["--defaultFieldOfView", "60"])
    case("exif_folder_focal", ["G1"], folder="folders/exif", args=["--defaultFocalLength", "8"])
    case("exif_folder_wide_fov", ["G6", "G8", "G13"], folder="folders/exif", args=["--defaultFieldOfView", "120"])
    case("exif_folder_global", ["N4"], folder="folders/exif", args=["--groupCameraFallback", "global"])
    case("exif_folder_image", ["N4"], folder="folders/exif", args=["--groupCameraFallback", "image"])
    case("exif_folder_sequence", ["W3"], folder="folders/exif", args=["--isSequence", "1"])
    case("exif_folder_ratio_offset", ["G5"], folder="folders/exif",
         args=["--defaultFocalRatio", "2", "--defaultOffsetX", "1.5", "--defaultOffsetY", "-2"])
    case("single_file_input", ["L5", "W2"], args=["--imageFolder", "{DATA}/img/canon50.jpg"] + single)
    case("single_file_needs_two", ["W2"], args=["--imageFolder", "{DATA}/img/canon50.jpg"], expect="fail",
         log=["At least two images"])
    case("single_file_unsupported", ["L6", "L8"], args=["--imageFolder", "{DATA}/img/notes.txt"], expect="fail",
         log=["is not a valid folder or file path"])
    case("nested_listing", ["L6", "L7"], folder="folders/nested", args=["--defaultFieldOfView", "45"])
    case("text_only_folder", ["L8"], folder="folders/textonly", expect="fail",
         log=["is not a valid folder or file path"])
    case("no_initialized_view", ["W2"], folder="folders/nested", expect="fail",
         log=["should have an initialized intrinsic"])
    case("viewid_filename", ["B4", "B5"], folder="folders/viewid",
         args=["--viewIdMethod", "filename", "--defaultFieldOfView", "45"])
    case("viewid_filename_nomatch", ["B5"], folder="folders/viewid_nonumber",
         args=["--viewIdMethod", "filename", "--defaultFieldOfView", "45"], expect="abort",
         log=["must match a unique number in the filename abc"])
    case("viewid_filename_badregex", ["B5"], folder="folders/viewid",
         args=["--viewIdMethod", "filename", "--viewIdRegex", "(", "--defaultFieldOfView", "45"],
         expect="abort", log=["Invalid regex conversion"])
    case("viewid_filename_nogroup", ["B5"], folder="folders/viewid",
         args=["--viewIdMethod", "filename", "--viewIdRegex", "\\d+", "--defaultFieldOfView", "45"],
         expect="abort", log=["must match a unique number"])
    case("viewid_filename_notnumber", ["B5"], folder="folders/viewid",
         args=["--viewIdMethod", "filename", "--viewIdRegex", "(img)", "--defaultFieldOfView", "45"],
         expect="abort", log=["can't be converted to a number"])
    case("viewid_metadata_variants", ["B4", "UID"], folder="folders/uid", args=["--defaultFieldOfView", "45"])

    # --- 3. rigs, frame ids, pose groups -----------------------------------------------------------------------------
    rig_args = ["--defaultFieldOfView", "45"]
    case("rig_ok", ["R1", "R3", "R7", "N6", "F1"], folder="folders/rig/ok", args=rig_args)
    case("rig_ok_serial", ["R1", "R3", "N1"], sfm_file="sfm/rig_serial.sfm", args=rig_args)
    case("rig_gap", ["R4"], folder="folders/rig/gap", args=rig_args, expect="fail",
         log=["Wrong subPoseId in rig structure"])
    case("rig_unequal", ["R6"], folder="folders/rig/unequal", args=rig_args,
         log=["Wrong number of poses per subPose"])
    case("rig_duplicate_frames", ["X1"], folder="folders/rig/dupframe", args=rig_args, expect="fail",
         log=["Multiple frames have the same frame id"])
    case("rig_no_number", ["R2"], folder="folders/rig/nonumber", args=rig_args, log=["Invalid rig structure"])
    case("rig_overflow", ["R2"], folder="folders/rig/overflow", args=rig_args, log=["Invalid rig structure"])
    case("frame_ids", ["F1"], folder="folders/frames", args=rig_args)
    case("pose_groups", ["P1", "P2", "P3"], folder="folders/posegroups", args=rig_args)

    # --- 4. raw / DCP -----------------------------------------------------------------------------------------------
    case("raw_default", ["D1", "D8"], folder="raw/one", args=single)
    case("raw_dcp_metadata", ["D2", "D4", "D7", "D8"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/full"])
    case("raw_dcp_linear_cache", ["D2", "D4", "D5", "D7"], folder="raw/two",
         args=["--rawColorInterpretation", "DcpLinearProcessing", "--colorProfileDatabase", "{DATA}/dcp/single"])
    case("raw_dcp_db_missing_error", ["D3", "C1"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata"], expect="fail",
         log=["does not exist or is empty", "Can't find color profile"])
    case("raw_dcp_db_empty_error", ["D3", "C1"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/empty"],
         expect="fail", log=["Can't find color profile for at least one input image"])
    case("raw_dcp_db_missing_noerror", ["D6", "D8", "C1"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--errorOnMissingColorProfile", "0"],
         log=["Can't find color profile"])
    case("raw_dcp_nomatch_noerror", ["D6", "D8", "C1"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/nomatch",
                        "--errorOnMissingColorProfile", "0"], log=["DCP Profile not found"])
    case("raw_dcp_nomake", ["D2"], folder="raw/nomake",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/full",
                        "--defaultFieldOfView", "45"])
    case("raw_dcp_single_illuminant", ["D7"], folder="raw/other",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/single"])
    case("raw_dcp_bad_tag_count", ["D9"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/badtags"],
         log=["Tag number out of range"])
    case("raw_dcp_bad_tag_type", ["D9"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/badtype"],
         log=["Invalid Tag"])
    case("raw_dcp_no_color_matrix", ["D9"], folder="raw/one",
         args=single + ["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/nomatrix"],
         log=["missing 'ColorMatrix1'"])
    case("raw_interpretation_none", ["D8"], folder="raw/one", args=single + ["--rawColorInterpretation", "None"])
    case("jpeg_dcp_ignored", ["D2"], folder="folders/exif",
         args=["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/full"])

    # --- 5. LCP ------------------------------------------------------------------------------------------------------
    case("lcp_single_file", ["LC1", "LC2", "LC13", "LC14", "G7", "G16"], folder="folders/exif",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/single/canon_ef50.lcp"])
    case("lcp_single_folder", ["LC1", "LC2"], folder="folders/exif",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/single"])
    case("lcp_multi", ["LC1", "LC3", "LC4", "LC5", "LC6", "LC7", "LC8", "LC9", "LC13", "LC14", "G7", "G14", "G16"],
         sfm_file="sfm/lcp_multi.sfm", args=["--lensCorrectionProfileInfo", "{DATA}/lcp/multi"])
    case("lcp_multi_use_model", ["LC6"], sfm_file="sfm/lcp_multi.sfm",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/multi",
               "--lensCorrectionProfileSearchIgnoreCameraModel", "0"])
    case("lcp_no_geometry", ["LC12"], folder="folders/exif",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/nogeometry"])
    case("lcp_fisheye", ["G7", "G14"], sfm_file="sfm/lcp_fisheye.sfm",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/fisheye"])
    case("lcp_altnames_element_style", ["LC11"], folder="folders/exif",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/altnames_element"], expect="abort", log=["stoi"],
         note="element-style LCP with AlternateLensNames: std::stoi throws during the full parse")
    case("lcp_broken", ["LC11"], folder="folders/exif", args=["--lensCorrectionProfileInfo", "{DATA}/lcp/broken"],
         expect="abort", log=["Invalid XML in LCP file"])

    # --- 6. sfm inputs: metadata that a JPEG can't carry ---------------------------------------------------------------
    case("sony", ["S6", "S9", "FL2"], sfm_file="sfm/sony.sfm")
    case("arri_dji", ["FL4", "FL5"], sfm_file="sfm/arri_dji.sfm")
    case("malformed_numbers", ["FL1", "S5", "G5", "N7"], sfm_file="sfm/malformed.sfm")
    case("aliases", ["MD"], sfm_file="sfm/aliases.sfm")
    case("existing_intrinsics", ["I1", "I2", "B1"], sfm_file="sfm/existing_intrinsics.sfm")
    case("sfm_pose_differs", ["B7"], sfm_file="sfm/pose_differs.sfm", log=["PoseId and viewId are different"])
    case("sfm_rig_without_pose", ["B6"], sfm_file="sfm/rig_without_pose.sfm", expect="abort",
         log=["marked as part of a rig"])
    case("sfm_future_version", ["L2", "L9"], sfm_file="sfm/future_version.sfm", expect="fail",
         log=["File has a version more recent than this library"])
    case("sfm_no_views", ["L9"], sfm_file="sfm/no_views.sfm", expect="fail", log=["Can't find views in input"])
    case("sfm_dangling_intrinsic", ["L4"], sfm_file="sfm/dangling_intrinsic.sfm", expect="abort",
         log=["cannot be found"])
    case("sfm_missing_image", ["B2"], sfm_file="sfm/missing_image.sfm", expect="abort",
         log=["Can't find/open image file"])
    case("sfm_malformed_json", ["L3"], sfm_file="sfm/malformed_json.sfm", expect="abort", log=["No such node"])
    case("sfm_bad_pixel_dimension", ["G2", "UID"], sfm_file="sfm/bad_pixel_dimension.sfm", expect="abort",
         log=["stoi"],
         note="std::stoi on a non-numeric PixelXDimension throws (uid.cpp / viewIO.cpp)")
    case("sfm_negative_sensor", ["S12"], sfm_file="sfm/negative_sensor.sfm")

    # --- 7. camera model and distortion options ----------------------------------------------------------------------
    for cam, dist in (("pinhole", "none"), ("pinhole", "radialk1"), ("pinhole", "radialk3"),
                      ("pinhole", "radialk3pt"), ("pinhole", "brown"), ("pinhole", "fisheye"),
                      ("pinhole", "fisheye1"), ("equidistant", "none"), ("equirectangular", "none")):
        case(f"model_{cam}_{dist}", ["A2", "G10", "G15"], sfm_file="sfm/models.sfm",
             args=["--defaultCameraModel", cam, "--defaultDistortionModel", dist])
    case("model_distortion_only", ["G9"], sfm_file="sfm/models.sfm", args=["--defaultDistortionModel", "radialk1"])

    # --- 8. additions found with the coverage report -----------------------------------------------------------------
    case("group_fallback_folder_explicit", ["N4"], folder="folders/exif", args=["--groupCameraFallback", "folder"])
    case("group_fallback_invalid", ["A1"], folder="folders/exif", args=["--groupCameraFallback", "camera"],
         expect="fail", log=["Invalid GroupCameraFallback type string camera"])
    case("uid_variants", ["UID"], sfm_file="sfm/uid_variants.sfm")
    case("edge_metadata", ["S1", "S5", "S8", "S9", "FL3", "FL4"], sfm_file="sfm/edge_metadata.sfm")
    case("pixel_dims", ["G2"], sfm_file="sfm/pixel_dims.sfm", log=["Resized image detected"])
    case("sfm_rig_with_pose", ["B7"], sfm_file="sfm/rig_with_pose.sfm")
    case("image_folder_not_a_folder", ["L6"], args=["--imageFolder", "/dev/null"], expect="fail",
         log=["is not a valid folder or file path"],
         platform="linux", note="exists, but is neither a regular file nor a directory")
    case("raw_dcp_rich_profiles", ["D4", "D7", "D9"], folder="raw/rich",
         args=["--rawColorInterpretation", "DcpMetadata", "--colorProfileDatabase", "{DATA}/dcp/rich"])
    case("lcp_attribute_style", ["LC1", "LC2", "LC13", "LC14", "G7", "G16"], folder="folders/exif",
         args=["--lensCorrectionProfileInfo", "{DATA}/lcp/attr/canon_attr.lcp"])
    for ext in ("ply", "baf", "abc", "usda"):
        case(f"output_format_{ext}", ["W4"], folder="folders/exif", args=["--output", "{OUT}/cameraInit." + ext])


def build_sfms():
    sony = [
        view(1, {"Make": "Sony", "Model": "ILME-FX6", "LensZoomActualFocalLength": "0.035",
                 "LensZoom35mmStillCameraEquivalent": "0.05"}),
        view(2, {"Make": "Sony", "Model": "ILME-FX6", "LensZoomActualFocalLength": "0.035",
                 "ImageSensorEffectiveWidth": "23500", "ImageSensorEffectiveHeight ": "15600"}),
        view(3, {"Make": "Sony", "Model": "ILME-FX6", "LensZoomActualFocalLength": "0.024",
                 "ImageSensorEffectiveWidth": "23500", "ImageSensorEffectiveHeight": "15600"}),
    ]
    sfm("sony", sony)
    sfm("arri_dji", [
        view(1, {"Make": "ARRI", "Model": "ALEXA Mini", "AxialNominalFocalLength": "35"}),
        view(2, {"Make": "ARRI", "Model": "ALEXA Mini", "AxialNominalFocalLength": "35000"}),
        view(3, {"Make": "DJI", "Model": "FC6310", "focal_length": "[50000,1000]"}),
        view(4, {"Make": "DJI", "Model": "FC6310", "focal_length": "[abc,1000]", "Exif:FocalLength": "8.8"}),
        view(5, {"Make": "DJI", "Model": "FC6310", "focal_length": "[noComma]", "Exif:FocalLength": "8.8"}),
    ])
    sfm("malformed", [
        view(1, {**M_CANON, "Exif:FocalLength": "24.5/2"}),
        view(2, {**M_CANON, "Exif:FocalLength": "1/0"}),
        view(3, {**M_CANON, "Exif:FocalLength": "n/a", "Exif:FocalLengthIn35mmFilm": "n/a"}),
        view(4, {**M_CANON, "Exif:FocalLength": "50", "PixelAspectRatio": "2"}),
        view(5, {**M_CANON, "Exif:FocalLength": "50", "PixelAspectRatio": "abc"}),
        view(6, {**M_CANON, "Exif:FocalLength": "50", "PixelAspectRatio": "-1"}),
        view(7, {"Exif:FocalLengthIn35mmFilm": "0", "Exif:FocalLength": "4"}),
        # empty first 35mm key: hasDigitMetadata skips it but getDoubleMetadata reads it (-1, then Sony x1000);
        # focal 5 keeps this view apart from view 7 (hashValue ignores the initialization mode)
        view(8, {"Exif:FocalLengthIn35mmFilm": "", "FocalLengthIn35mmFilm": "30", "Exif:FocalLength": "5"}),
    ])
    sfm("aliases", [
        view(1, {"cameraMake": "Canon", "cameraModel": "Canon EOS 5D", "focalLength": "50", "cameraSerialNumber": "9",
                 "lensSerialNumber": "8", "lensModel": "EF50mm f/1.8", "FNumber": "abc", "ApertureValue": "3"}),
        view(2, {"tiff:Make": "Canon", "camera model name": "Canon EOS 5D", "lens_focal_length": "35",
                 "Serial Number": "7"}),
        view(3, {"camera make": "GoPro", "camera_model": "GoPro HERO9", "camera_focal": "3"}),
    ])
    pinhole_ok = dict(INTRINSIC_TEMPLATE, intrinsicId="77")
    pinhole_nofocal = dict(INTRINSIC_TEMPLATE, intrinsicId="78", focalLength="0", initialFocalLength="-1")
    equidistant = dict(INTRINSIC_TEMPLATE, intrinsicId="79", type="equidistant", distortionType="none",
                       distortionParams="", focalLength="20", initialFocalLength="20")
    sfm("existing_intrinsics", [
        view(1, M_CANON | {"Exif:FocalLength": "50"}, intrinsicId=77),
        view(2, M_CANON | {"Exif:FocalLength": "50"}, intrinsicId=78),
        view(3, M_CANON | {"Exif:FocalLength": "50"}, intrinsicId=79),
        view(4, M_CANON | {"Exif:FocalLength": "50"}, intrinsicId=77, poseId=4, width=16, height=12),
    ], intrinsics=[pinhole_ok, pinhole_nofocal, equidistant])
    sfm("pose_differs", [view(1, M_CANON | {"Exif:FocalLength": "50"}, poseId=99),
                         view(2, M_CANON | {"Exif:FocalLength": "50"})])
    sfm("rig_without_pose", [view(1, M_CANON | {"Exif:FocalLength": "50"}, rigId=5, subPoseId=0),
                             view(2, M_CANON | {"Exif:FocalLength": "50"})])
    sfm("rig_serial", [view(10 + 2 * s + f, M_CANON | {"Exif:FocalLength": "50", "Exif:BodySerialNumber": f"B{s}"},
                            path=f"folders/rig/ok/rig/{s}/frame_000{f + 1}.jpg")
                       for s in (0, 1) for f in (0, 1)])
    sfm("future_version", [view(1, M_CANON)], version=["99", "0", "0"])
    sfm("no_views", [])
    sfm("dangling_intrinsic", [view(1, M_CANON, intrinsicId=1234)])
    sfm("missing_image", [view(1, M_CANON | {"Exif:FocalLength": "50"}, path="img/does_not_exist.jpg")])
    sfm("malformed_json", [], raw="{ \"version\": [\"1\", \"2\", \"14\"], \"views\": [ { \"viewId\": \"1\" } ] }\n")
    sfm("bad_pixel_dimension", [view(1, M_CANON | {"Exif:FocalLength": "50", "Exif:PixelXDimension": "abc",
                                                   "Exif:PixelYDimension": "12"})])
    sfm("negative_sensor", [view(1, {"Make": "Negative", "Model": "Width", "Exif:FocalLength": "50"}),
                            view(2, {"Make": "Foo", "Model": "Bar", "Exif:FocalLength": "50",
                                     "ImageSensorEffectiveWidth": "-5"})])
    sfm("edge_metadata", [
        view(1, {"Model": "Canon EOS 5D", "Exif:FocalLength": "50"}),                         # model without make
        view(2, {"Make": "Negative", "Model": "Two", "Exif:FocalLengthIn35mmFilm": "50"}),    # db width -2
        view(3, {"Make": "Testcam", "Model": "Model One", "Exif:FocalLength": "20",
                 "ImageSensorEffectiveWidth": "9000"}),                                      # db hit wins over Sony
        view(4, {**M_CANON, "Exif:FocalLength": "32767"}),
        view(5, {**M_CANON, "Exif:FocalLength": "4294967295"}),
        view(6, {**M_CANON, "Exif:FocalLength": "2147483647"}),
        view(7, {**M_CANON, "Exif:FocalLength": "4294970"}),
        view(8, {"Make": "DJI", "Model": "FC6310", "focal_length": "x[50000,1000]", "Exif:FocalLength": "9"}),
        view(9, {"Make": "DJI", "Model": "FC6310", "focal_length": "[50000,1000]x", "Exif:FocalLength": "9"}),
    ])
    dims = {**M_CANON, "Exif:FocalLength": "50"}
    sfm("pixel_dims", [
        view(1, dims | {"Exif:PixelXDimension": "32"}),
        view(2, dims | {"Exif:PixelYDimension": "24"}),
        view(3, dims | {"Exif:PixelXDimension": "12", "Exif:PixelYDimension": "20"}),
        view(4, dims | {"Exif:PixelXDimension": "32", "Exif:PixelYDimension": "0"}),
        view(5, dims | {"Exif:PixelXDimension": "16", "Exif:PixelYDimension": "10"}),
    ])
    sfm("rig_with_pose", [view(1, M_CANON | {"Exif:FocalLength": "50"}, rigId=5, subPoseId=0, poseId=100),
                          view(2, M_CANON | {"Exif:FocalLength": "50"})])
    sfm("models", [view(1, M_CANON | {"Exif:FocalLength": "50"}),
                   view(2, {"Make": "GoPro", "Model": "GoPro HERO9", "Exif:FocalLength": "3"})])
    # Views that share an intrinsic must agree on the LCP distortion: IntrinsicBase::hashValue() ignores the
    # distortion parameters, so mixing them in one group makes the stored distortion depend on thread order.
    # Hence one focal length per LCP outcome.
    canon_lens = {**M_CANON, "Exif:FocalLength": "35", "FNumber": "4"}
    sfm("lcp_multi", [
        view(1, canon_lens | {"Exif:LensModel": "EF50mm f/1.8"}),                      # raw profile, LensID -1
        view(2, canon_lens | {"Exif:LensModel": "EF50mm f/1.8"}),                      # same key: mapping cache
        view(3, canon_lens | {"Exif:LensModel": "EF50mm f/1.8", "Exif:LensID": "61"}),  # second lens id
        view(4, canon_lens | {"Exif:LensModel": "EF50mm f/1.8 II", "Exif:FocalLength": "36"}),  # non-raw only
        view(5, canon_lens | {"Exif:LensModel": "EF8-15mm f/4L Fisheye", "Exif:FocalLength": "12"}),  # fisheye
        view(6, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "10", "FNumber": "1"}),
        view(7, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "90"}),  # above all focals
        view(8, canon_lens | {"Exif:LensModel": "Unknown lens", "Exif:FocalLength": "37"}),  # not found
        view(9, canon_lens | {"Exif:LensModel": "Unknown lens", "Exif:FocalLength": "37"}),  # not found, cached
        view(10, {"Make": "Canon", "Model": "Canon EOS 5D", "Exif:FocalLength": "38"}),  # no lens model
        view(11, {"Make": "Nikon", "Model": "Nikon D800", "Exif:FocalLength": "50", "Exif:LensModel": "AF-S 50mm"}),
        view(12, canon_lens | {"Make": "Custom", "Exif:LensModel": "EF50mm f/1.8"}),
        view(13, {"Make": "Canon", "Model": "Canon EOS 5D", "Exif:LensModel": "EF50mm f/1.8"}),  # no focal
        # vignetting search (aperture = 2*log2(FNumber); FNumber 1 means aperture 0, i.e. "unknown")
        view(14, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "24", "FNumber": "1"}),
        view(15, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "40", "FNumber": "16"}),
        view(16, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "41", "FNumber": "1"}),
        view(17, canon_lens | {"Exif:LensModel": "EF28-70 Odd", "Exif:FocalLength": "42", "FNumber": "2.83"}),
        view(18, canon_lens | {"Exif:LensModel": "EF 100 Macro USM", "Exif:FocalLength": "13"}),  # alternate name
        view(19, canon_lens | {"Exif:LensModel": "EF35mm f/2", "Exif:FocalLength": "43", "Exif:LensID": "abc"}),
        view(20, canon_lens | {"Exif:LensModel": "EF Mixed", "Exif:FocalLength": "44"}),
        view(21, canon_lens | {"Exif:LensModel": "EF 85 Alt", "Exif:FocalLength": "45"}),  # name listed after ids
        view(22, canon_lens | {"Exif:LensModel": "EF85mm", "Exif:FocalLength": "46", "Exif:LensID": "300"}),
        view(23, canon_lens | {"Exif:LensModel": "EF Wrong", "Exif:FocalLength": "47"}),
        view(24, canon_lens | {"Exif:LensModel": "EF Pretty", "Exif:FocalLength": "48"}),
    ])
    # views without viewId: computeViewUID from metadata (one image file per view keeps the keys stable)
    for name in ("counter", "subsec", "dims"):
        copy("img/base.jpg", f"img/uid/{name}.jpg")
    sfm("uid_variants", [
        {"path": "{DATA}/img/uid/counter.jpg", "metadata": {"imageCounter": "42", "Exif:FocalLength": "50"}},
        {"path": "{DATA}/img/uid/subsec.jpg", "metadata": {"Exif:DateTimeOriginal": "2024:01:01 00:00:00",
                                                           "Exif:SubsecTimeOriginal": "5",
                                                           "Exif:BodySerialNumber": "S1", "Exif:FocalLength": "50"}},
        {"path": "{DATA}/img/uid/dims.jpg", "metadata": {"Exif:PixelXDimension": "16", "Exif:PixelYDimension": "12",
                                                         "ImageUniqueID": "abc", "Exif:FocalLength": "50"}},
    ])
    sfm("lcp_fisheye", [view(1, M_CANON | {"Exif:FocalLength": "35", "FNumber": "4"}),
                        view(2, {"Make": "Custom", "Model": "X", "Exif:FocalLength": "35"})])


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if DATA.exists():
        shutil.rmtree(DATA)
    DATA.mkdir()
    build_images()
    build_exif_folder()
    build_listing_folders()
    build_rig_folders()
    build_posegroup_folders()
    build_raw()
    build_lcp()
    write_sensor_db()
    (DATA / "root/share/aliceVision").mkdir(parents=True)
    shutil.copyfile(DATA / "sensors.db", DATA / "root/share/aliceVision/cameraSensors.db")
    build_sfms()
    declare_cases()
    names = [c["name"] for c in CASES]
    assert len(names) == len(set(names)), "duplicate case names"
    (ROOT / "cases.json").write_text(json.dumps(CASES, indent=1) + "\n")
    size = sum(p.stat().st_size for p in DATA.rglob("*") if p.is_file())
    logger.info("%d cases, data/ is %.1f KB", len(CASES), size / 1024)


if __name__ == "__main__":
    main()
