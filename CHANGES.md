# AliceVision Changelog

## Release 3.2.0 (2023/12/07)

### Major Features

- New software for intrinsics and rig calibration using a multiview acquisition of a checkerboard [PR](https://github.com/alicevision/AliceVision/pull/1524)
- New semantic image segmentation module [PR](https://github.com/alicevision/AliceVision/pull/1476)

### Features

- Support pixel aspect ratio [PR](https://github.com/alicevision/AliceVision/pull/1448)
- Automatically align the sfm result [PR](https://github.com/alicevision/AliceVision/pull/1546)
- New pipeline for pure rotation tracking [PR](https://github.com/alicevision/AliceVision/pull/1548)
- KeyframeSelection: Add support for masks [PR](https://github.com/alicevision/AliceVision/pull/1518)

### Other Improvements

- Start Development 3.2 [PR](https://github.com/alicevision/AliceVision/pull/1479)
- Split SFMData between reconstructed and other views [PR](https://github.com/alicevision/AliceVision/pull/1409)
- Noise reduction in HDR merging [PR](https://github.com/alicevision/AliceVision/pull/1469)
- Add a new optional command to export textured mesh to a USD format [PR](https://github.com/alicevision/AliceVision/pull/1455)
- Enable multiple calibrations on hdr [PR](https://github.com/alicevision/AliceVision/pull/1484)
- [Panorama] Enforce priors after estimation [PR](https://github.com/alicevision/AliceVision/pull/1371)
- tolerant bracket size estimation [PR](https://github.com/alicevision/AliceVision/pull/1488)
- Make sure openexr does not use too many threads [PR](https://github.com/alicevision/AliceVision/pull/1489)
- HDR default number of brackets should be 0 [PR](https://github.com/alicevision/AliceVision/pull/1490)
- Remove limits on outliers for brackets detection [PR](https://github.com/alicevision/AliceVision/pull/1491)
- Correctly count the best brackets count [PR](https://github.com/alicevision/AliceVision/pull/1492)
- Compositing does not fail on range error [PR](https://github.com/alicevision/AliceVision/pull/1498)
- [software] panoramaPostProcessing: export downscaled panorama levels [PR](https://github.com/alicevision/AliceVision/pull/1496)
- [meshing] export computed bounding box [PR](https://github.com/alicevision/AliceVision/pull/1473)
- [software] sfmTransform: Increase precision of coefficients in the log [PR](https://github.com/alicevision/AliceVision/pull/1486)
- Limit EXR multi-threading for texturing [PR](https://github.com/alicevision/AliceVision/pull/1509)
- [fuseCut] Fix adding helper points [PR](https://github.com/alicevision/AliceVision/pull/1465)
- add memory use constraint in texturing [PR](https://github.com/alicevision/AliceVision/pull/1511)
- Manage LCP in imageProcessing [PR](https://github.com/alicevision/AliceVision/pull/1459)
- Camera exposure update [PR](https://github.com/alicevision/AliceVision/pull/1508)
- Masking: Handle different file extensions for masks and allow mask inversion in `ImageSegmentation` [PR](https://github.com/alicevision/AliceVision/pull/1517)
- [keyframe] Do not fail when there are consecutive missing frames for the smart Keyframe Selection method [PR](https://github.com/alicevision/AliceVision/pull/1510)
- Keyframe Selection: Add multi-threading and improve overall performances [PR](https://github.com/alicevision/AliceVision/pull/1512)
- [camera] refactorize and clean up code [PR](https://github.com/alicevision/AliceVision/pull/1515)
- Meshing: fix in graphcut weighting and minor sampling improvements [PR](https://github.com/alicevision/AliceVision/pull/1520)
- bundle adjustment improvement [PR](https://github.com/alicevision/AliceVision/pull/1477)
- Add automatic method for HDR calibration [PR](https://github.com/alicevision/AliceVision/pull/1521)
- [sfmTransform] Rewrite check on ambiguity in sfmTransform [PR](https://github.com/alicevision/AliceVision/pull/1525)
- [image] ImageCache: refine mutex usage [PR](https://github.com/alicevision/AliceVision/pull/1504)
- Pose3 internal representation [PR](https://github.com/alicevision/AliceVision/pull/1472)
- [SFM] New incremental logic for sfm [PR](https://github.com/alicevision/AliceVision/pull/1377)
- ImageInfo class for sfmData [PR](https://github.com/alicevision/AliceVision/pull/1533)
- Clean symbolic bundle code [PR](https://github.com/alicevision/AliceVision/pull/1531)
- fix flann randomSeed [PR](https://github.com/alicevision/AliceVision/pull/1530)
- SFM bootstraping [PR](https://github.com/alicevision/AliceVision/pull/1432)
- PanoramaPostProcessing: Use the generated panorama's name for the downscaled pyramid levels [PR](https://github.com/alicevision/AliceVision/pull/1542)
- Fix View copy constructor and remove unused control points [PR](https://github.com/alicevision/AliceVision/pull/1536)
- [Cleanup] Remove pointers [PR](https://github.com/alicevision/AliceVision/pull/1549)
- [DSPSIFT] Change orientation value [PR](https://github.com/alicevision/AliceVision/pull/1551)
- Code cleanup [PR](https://github.com/alicevision/AliceVision/pull/1556)
- Deal with multiple brackets with same score [PR](https://github.com/alicevision/AliceVision/pull/1493)
- Remove unused gamma functions [PR](https://github.com/alicevision/AliceVision/pull/1516)
- [clean] Ransac containers [PR](https://github.com/alicevision/AliceVision/pull/1566)
- Cleaning up ACRansac [PR](https://github.com/alicevision/AliceVision/pull/1573)
- Input color space setting [PR](https://github.com/alicevision/AliceVision/pull/1571)
- Fallback when camera motion is not distinctive enough [PR](https://github.com/alicevision/AliceVision/pull/1563)
- [sfmData] imageInfo: Retrieve focal length when it's available in non-standard metadata keys [PR](https://github.com/alicevision/AliceVision/pull/1583)
- Reformat with clang [PR](https://github.com/alicevision/AliceVision/pull/1580)
- Binary format type for ply IO [PR](https://github.com/alicevision/AliceVision/pull/1576)
- Automatic reorientation [PR](https://github.com/alicevision/AliceVision/pull/1585)
- Fov compute for intrinsics [PR](https://github.com/alicevision/AliceVision/pull/1588)
- Upgrade tracks to allow more advanced storage and reduce indirections [PR](https://github.com/alicevision/AliceVision/pull/1594)
- Color space conversion update [PR](https://github.com/alicevision/AliceVision/pull/1591)
- fixed unnecessary descriptor load for sequential image matching [PR](https://github.com/alicevision/AliceVision/pull/1599)
- [sfm] SequentialSfM: Initialize the resection ID in the constructor and set it for the initial pair [PR](https://github.com/alicevision/AliceVision/pull/1593)
- Add ancestor images info in view [PR](https://github.com/alicevision/AliceVision/pull/1582)
- Intrinsics update after auto reorientation [PR](https://github.com/alicevision/AliceVision/pull/1595)
- [sfmData] Ensure copies of SfMData are deep instead of shallow [PR](https://github.com/alicevision/AliceVision/pull/1604)
- New utility nodes to create camera rigs and merge two sfmData [PR](https://github.com/alicevision/AliceVision/pull/1568)

### Bugfixes

- [segmentation] fix windows build [PR](https://github.com/alicevision/AliceVision/pull/1480)
- [segmentation] fix crash when resized image is too small [PR](https://github.com/alicevision/AliceVision/pull/1485)
- Bugfix on dng reading [PR](https://github.com/alicevision/AliceVision/pull/1502)
- [sfmDataIO] getViewIntrinsics corrections [PR](https://github.com/alicevision/AliceVision/pull/1522)
- [Fix] transformsRt function [PR](https://github.com/alicevision/AliceVision/pull/1541)
- [software] imageSegmentation: read images in sRGB colorspace [PR](https://github.com/alicevision/AliceVision/pull/1543)
- [software] LdrToHdrMerge: fix bug introduced in PR 1536 [PR](https://github.com/alicevision/AliceVision/pull/1557)
- [sfmDataIO] restore focal ratio support [PR](https://github.com/alicevision/AliceVision/pull/1559)
- [build][windows] fix for OpenMP error causing build issue on Windows [PR](https://github.com/alicevision/AliceVision/pull/1552)
- [sfmDataIO] remove declared/referenced poses validation [PR](https://github.com/alicevision/AliceVision/pull/1529)
- getSensorSize update [PR](https://github.com/alicevision/AliceVision/pull/1519)
- [sfmDataIO] Fix unhandled exception when parsing images with large numbers in the filenames [PR](https://github.com/alicevision/AliceVision/pull/1565)
- [depthMap] DepthMapEstimator: Correctly limit the number of simultaneous RCs [PR](https://github.com/alicevision/AliceVision/pull/1569)
- [sfm] add try-catch for feature/region loading [PR](https://github.com/alicevision/AliceVision/pull/1575)
- [fuseCut] Fix export debug mesh [PR](https://github.com/alicevision/AliceVision/pull/1538)
- Fixup panoramaInit manual init [PR](https://github.com/alicevision/AliceVision/pull/1577)
- [keyframe] fix intrinsics UID [PR](https://github.com/alicevision/AliceVision/pull/1579)
- Fix: Add missing 3DEAnamorphic4 camera type [PR](https://github.com/alicevision/AliceVision/pull/1581)
- Bugfix when checking raw format [PR](https://github.com/alicevision/AliceVision/pull/1572)
- camera Lock is correctly casted to int in json [PR](https://github.com/alicevision/AliceVision/pull/1584)
- Sequential SfM: Replayable initial pair [PR](https://github.com/alicevision/AliceVision/pull/1592)
- [sfm] Sequential ReconstructionEngine: Use SfM Params' `nbFirstUnstableCameras` instead of a constant [PR](https://github.com/alicevision/AliceVision/pull/1602)
- [sfmData] Add const accessor for getView [PR](https://github.com/alicevision/AliceVision/pull/1606)
- [build] Add an option to disable the use of the GPU with ONNX [PR](https://github.com/alicevision/AliceVision/pull/1608)
- fix applyCalibration [PR](https://github.com/alicevision/AliceVision/pull/1609)
- Fix errors on sfm with applyCalibration [PR](https://github.com/alicevision/AliceVision/pull/1613)
- fix sfm with undistortion [PR](https://github.com/alicevision/AliceVision/pull/1615)
- [mesh] Texturing fix: specify a name for the materials to avoid issues in Maya [PR](https://github.com/alicevision/AliceVision/pull/1617)

### CI, Build and Documentation

- [ci] Update GitHub actions for the Continuous Integration on Windows [PR](https://github.com/alicevision/AliceVision/pull/1483)
- [docker] Update version of the AV dependencies image [PR](https://github.com/alicevision/AliceVision/pull/1482)
- [doc] INSTALL.md: Fix typo in "nonfree" module for OpenCV in vcpkg install [PR](https://github.com/alicevision/AliceVision/pull/1487)
- [build] Download, copy and export the semantic segmentation model [PR](https://github.com/alicevision/AliceVision/pull/1481)
- Remove internal flann [PR](https://github.com/alicevision/AliceVision/pull/1495)
- [build] Update OIIO to 2.4.13.0 [PR](https://github.com/alicevision/AliceVision/pull/1527)
- Fixes for osx [PR](https://github.com/alicevision/AliceVision/pull/1463)
- Make sure one can deactivate part of modules [PR](https://github.com/alicevision/AliceVision/pull/1534)
- [ci] Windows: Stop building modules that are not tested [PR](https://github.com/alicevision/AliceVision/pull/1537)
- [CI] Don't run CI on draft PR [PR](https://github.com/alicevision/AliceVision/pull/1553)
- [docker] Add Python in the Docker image of the dependencies [PR](https://github.com/alicevision/AliceVision/pull/1540)
- [docker] Dependencies: Remove duplicated `gcc` install  [PR](https://github.com/alicevision/AliceVision/pull/1564)
- [ci] Update vcpkg archive with boost-geometry and liblemon [PR](https://github.com/alicevision/AliceVision/pull/1561)
- [ci] Update functional tests [PR](https://github.com/alicevision/AliceVision/pull/1535)
- Remove lemon from internal dependencies [PR](https://github.com/alicevision/AliceVision/pull/1547)
- Clean dependencies in CMakeLists [PR](https://github.com/alicevision/AliceVision/pull/1570)
- Remove lemon from public links [PR](https://github.com/alicevision/AliceVision/pull/1587)
- Run CI on draft pull requests [PR](https://github.com/alicevision/AliceVision/pull/1600)

### Updates to the Sensor Database

- Added Zed2i StereoLabs Zed2i sensor info [PR](https://github.com/alicevision/AliceVision/pull/1359)
- Update cameraSensors.db [PR](https://github.com/alicevision/AliceVision/pull/1528)

### Contributors

[almarouk](https://github.com/almarouk), [cbentejac](https://github.com/cbentejac), [demoulinv](https://github.com/demoulinv), [emanuelenencioni](https://github.com/emanuelenencioni), [fabiencastan](https://github.com/fabiencastan), [Kokika](https://github.com/Kokika), [mh0g](https://github.com/mh0g), [mugulmd](https://github.com/mugulmd), [serguei-k](https://github.com/serguei-k), [servantftechnicolor](https://github.com/servantftechnicolor), [simogasp](https://github.com/simogasp), [unageek](https://github.com/unageek)


## Release 3.1.0 (2023/06/21)

### Major Features

- New Photometric Stereo software [PR](https://github.com/alicevision/AliceVision/pull/999)
- [calibration] Checkerboard detection [PR](https://github.com/alicevision/AliceVision/pull/1331)
- [depthMap] New option for multi-resolution similarity estimation and optimizations [PR](https://github.com/alicevision/AliceVision/pull/1351)
- [utils] split360Images: support SfMData input and output [PR](https://github.com/alicevision/AliceVision/pull/1382)
- [sfmTransform] Creation of auto mode [PR](https://github.com/alicevision/AliceVision/pull/1402)
- Distortion calibration [PR](https://github.com/alicevision/AliceVision/pull/1415)

### Features

- Add rec709 color space definition [PR](https://github.com/alicevision/AliceVision/pull/1412)
- SfM: Expose new parameters to the command line [PR](https://github.com/alicevision/AliceVision/pull/1413)
- [image] New image cache [PR](https://github.com/alicevision/AliceVision/pull/1310)
- [sfmTransform] rewrite auto_from_cameras [PR](https://github.com/alicevision/AliceVision/pull/1376)
- Stitching color space [PR](https://github.com/alicevision/AliceVision/pull/1380)
- [panoramaInit]  add generation of contact sheet using xml input [PR](https://github.com/alicevision/AliceVision/pull/1392)
- Add compression option for exr and jpg images [PR](https://github.com/alicevision/AliceVision/pull/1408)
- [cmake] Update CXX standard from 14 to 17 [PR](https://github.com/alicevision/AliceVision/pull/1428)
- ColorTemperatureAsImageProcessingParameter [PR](https://github.com/alicevision/AliceVision/pull/1433)

### Other Improvements

- Start Development Version 3.1.0 [PR](https://github.com/alicevision/AliceVision/pull/1401)
- [panorama] compositing common scale selection [PR](https://github.com/alicevision/AliceVision/pull/1394)
- Move cmdline from system to dedicated module [PR](https://github.com/alicevision/AliceVision/pull/1410)
- [cameraInit] Reject input if multiple frames have same frame id [PR](https://github.com/alicevision/AliceVision/pull/1259)
- Keyframe Selection: Add support for SfMData files as inputs and outputs [PR](https://github.com/alicevision/AliceVision/pull/1406)
- [panorama] Generate a preview panorama [PR](https://github.com/alicevision/AliceVision/pull/1388)
- add tracksbuilder app [PR](https://github.com/alicevision/AliceVision/pull/1418)
- HDR filenames [PR](https://github.com/alicevision/AliceVision/pull/1424)
- IncrementalSfM: expose nbOutliersThreshold param [PR](https://github.com/alicevision/AliceVision/pull/1436)
- Fixed separator in CMAKE_INSTALL_RPATH [PR](https://github.com/alicevision/AliceVision/pull/1427)
- oiio gammaX.Y color spaces management [PR](https://github.com/alicevision/AliceVision/pull/1445)
- Exposure and format adjustment [PR](https://github.com/alicevision/AliceVision/pull/1414)
- Ground level alignment based on sfm point cloud [PR](https://github.com/alicevision/AliceVision/pull/1438)
- [image] ImageCache improvements [PR](https://github.com/alicevision/AliceVision/pull/1419)
- [software] applyCalibration: simply copy input data when calibrated data filename is empty [PR](https://github.com/alicevision/AliceVision/pull/1450)
- [gitignore] Update .gitignore [PR](https://github.com/alicevision/AliceVision/pull/1453)
- [software] Do not fail if range start is larger than the number of views [PR](https://github.com/alicevision/AliceVision/pull/1457)
- KeyframeSelection: Add new parameter value to disable the export of keyframes [PR](https://github.com/alicevision/AliceVision/pull/1454)
- Disable libraw flip  [PR](https://github.com/alicevision/AliceVision/pull/1301)
- Stereo Photometry: Add robustness to unexpected inputs [PR](https://github.com/alicevision/AliceVision/pull/1452)
- Set LibRaw options [PR](https://github.com/alicevision/AliceVision/pull/1468)

### Bugfixes, CI, Build and Documentation

- [cmake] housekeeping [PR](https://github.com/alicevision/AliceVision/pull/1407)
- [software] ldrTohdrSampling: fix error when using jpg, rawColorInterpretation was wrongly interpreted [PR](https://github.com/alicevision/AliceVision/pull/1420)
- [doc] RELEASING: Add example command to generate the release note [PR](https://github.com/alicevision/AliceVision/pull/1421)
- Build: Add dependency to Boost.JSON and make the Calibration module Boost1.81-compliant [PR](https://github.com/alicevision/AliceVision/pull/1416)
- Fix command line parsing for executables that used the old one [PR](https://github.com/alicevision/AliceVision/pull/1422)
- Fix small compilation issue in trackIO on Windows [PR](https://github.com/alicevision/AliceVision/pull/1429)
- [pipeline] cameraInit: fix rigHasUniqueFrameIds [PR](https://github.com/alicevision/AliceVision/pull/1430)
- Fix .clang-format after updating CXX standard [PR](https://github.com/alicevision/AliceVision/pull/1446)
- [doc] INSTALL.md: remove geogram broken link [PR](https://github.com/alicevision/AliceVision/pull/1451)
- [Meshing] Fix to avoid small holes in the final mesh [PR](https://github.com/alicevision/AliceVision/pull/1447)
- [sfmDataIO] Alembic Import: initialize intrinsics properly [PR](https://github.com/alicevision/AliceVision/pull/1456)
- [ci] Update docker image for dependencies to fix runtime issue in CI [PR](https://github.com/alicevision/AliceVision/pull/1458)
- Debug in camera object [PR](https://github.com/alicevision/AliceVision/pull/1461)
- Panorama was using too much memory for no reason [PR](https://github.com/alicevision/AliceVision/pull/1462)
- [mvsUtils] fix writing metadata for filtered depthmaps [PR](https://github.com/alicevision/AliceVision/pull/1467)
- [docker] Download, copy and export the sphere detection model [PR](https://github.com/alicevision/AliceVision/pull/1470)
- [image] readImageSpec: perform safety check before anything else [PR](https://github.com/alicevision/AliceVision/pull/1474)
- CameraPose lock was not correctly loaded [PR](https://github.com/alicevision/AliceVision/pull/1475)

### Updates to the Sensor Database

- Update cameraSensors.db [PR](https://github.com/alicevision/AliceVision/pull/1442)
- Add Google Pixel 7/7 Pro main camera to cameraSensors.db [PR](https://github.com/alicevision/AliceVision/pull/1444)
- Update cameraSensors.db [PR](https://github.com/alicevision/AliceVision/pull/1443)

### Contributors

[almarouk](https://github.com/almarouk), [cbentejac](https://github.com/cbentejac), [demoulinv](https://github.com/demoulinv), [earlywill](https://github.com/earlywill), [erikjwaxx](https://github.com/erikjwaxx), [fabiencastan](https://github.com/fabiencastan), [gregoire-dl](https://github.com/gregoire-dl), [ICIbrahim](https://github.com/ICIbrahim), [jmelou](https://github.com/jmelou), [mugulmd](https://github.com/mugulmd), [serguei-k](https://github.com/serguei-k), [servantftechnicolor](https://github.com/servantftechnicolor), [simogasp](https://github.com/simogasp)


## Release 3.0.0 (2023/03/20)

### Release Notes Summary

- Depth map improvements [PR](https://github.com/alicevision/AliceVision/pull/1296)
- Depth map refactoring [PR](https://github.com/alicevision/AliceVision/pull/619)
- RAW advanced processing [PR](https://github.com/alicevision/AliceVision/pull/1368)
- Color management for RAW images [PR](https://github.com/alicevision/AliceVision/pull/1180)
- Color Space management [PR](https://github.com/alicevision/AliceVision/pull/1170)
- Output color space in ACES or ACEScg [PR](https://github.com/alicevision/AliceVision/pull/1169)
- Update panorama pipeline for very large panoramas [PR](https://github.com/alicevision/AliceVision/pull/1244)
- [Feature Matching] Add an option to remove matches without enough motion [PR](https://github.com/alicevision/AliceVision/pull/1198)
- GPS alignment from exif metadata [PR](https://github.com/alicevision/AliceVision/pull/1069)
- Generate NormalMaps and HeightMaps [PR](https://github.com/alicevision/AliceVision/pull/1092)
- Use assimp as mesh importer and exporter [PR](https://github.com/alicevision/AliceVision/pull/1090)
- Integration of AprilTag library [PR](https://github.com/alicevision/AliceVision/pull/950)
- New node to import known poses for various file formats [PR](https://github.com/alicevision/AliceVision/pull/1078)
- New ImageMasking and MeshMasking software and some cameras rig improvements [PR](https://github.com/alicevision/AliceVision/pull/1083)

### Other Improvements and Bug Fixes

- Fix crash when output extension is missing in image processing. [PR](https://github.com/alicevision/AliceVision/pull/1395)
- Fix reading non raw image format in imageProcessing. [PR](https://github.com/alicevision/AliceVision/pull/1381)
- [sfmTransform] fix issue on referenceView if some poses are missing [PR](https://github.com/alicevision/AliceVision/pull/1374)
- Texturing Color Space [PR](https://github.com/alicevision/AliceVision/pull/1379)
- [software] fix split dual fisheye [PR](https://github.com/alicevision/AliceVision/pull/1378)
- Keyframe Selection: Rework and add new selection methods [PR](https://github.com/alicevision/AliceVision/pull/1343)
- Add support for Lens Camera Profiles (LCP) [PR](https://github.com/alicevision/AliceVision/pull/1215)
- Add missing build option to build assimp with gltf [PR](https://github.com/alicevision/AliceVision/pull/1370)
- [panorama] force pyramid levels count in compositing [PR](https://github.com/alicevision/AliceVision/pull/1369)
- [Panorama] New option to disable compositing tiling [PR](https://github.com/alicevision/AliceVision/pull/1367)
- [panorama] Propagate metadatas [PR](https://github.com/alicevision/AliceVision/pull/1361)
- Panorama alignment with a reference camera [PR](https://github.com/alicevision/AliceVision/pull/1334)
- [image] io: add missing check in isRawFormat [PR](https://github.com/alicevision/AliceVision/pull/1354)
- HDR luminance statistics bugfix [PR](https://github.com/alicevision/AliceVision/pull/1353)
- Add option to apply DCP metadata in imageProcessing [PR](https://github.com/alicevision/AliceVision/pull/1340)
- [hdr] Compute the center exposure of the hdr automatically [PR](https://github.com/alicevision/AliceVision/pull/1315)
- add missing config.hpp include [PR](https://github.com/alicevision/AliceVision/pull/1341)
- Views ancestors for hdr [PR](https://github.com/alicevision/AliceVision/pull/1337)
- [software] New triangulation tool [PR](https://github.com/alicevision/AliceVision/pull/1314)
- DCP error management [PR](https://github.com/alicevision/AliceVision/pull/1328)
- [software] remove "utils" from executables names [PR](https://github.com/alicevision/AliceVision/pull/1318)
- read image with missing cam_mul metadata [PR](https://github.com/alicevision/AliceVision/pull/1323)
- Raw Images: minor adjustments [PR](https://github.com/alicevision/AliceVision/pull/1320)
- Raw Images: minor update [PR](https://github.com/alicevision/AliceVision/pull/1317)
- Add NLMeans denoiser in ImageProcessing [PR](https://github.com/alicevision/AliceVision/pull/1181)
- [system] minor fix on HardwareContext log [PR](https://github.com/alicevision/AliceVision/pull/1313)
- Fix and harmonize command lines' descriptions [PR](https://github.com/alicevision/AliceVision/pull/1311)
- Allow parameters to limit the number of Cores used and Memory available information [PR](https://github.com/alicevision/AliceVision/pull/1304)
- Updates for compatibility with recent ceres versions [PR](https://github.com/alicevision/AliceVision/pull/1305)
- [sfmColorHarmonize] update selection method enum and command-line argument [PR](https://github.com/alicevision/AliceVision/pull/1306)
- Factorize command line code for utils and export apps [PR](https://github.com/alicevision/AliceVision/pull/1303)
- Replace command line parsing code with a class to factorize [PR](https://github.com/alicevision/AliceVision/pull/1302)
- [sfm] make sure we give other chances to candidates [PR](https://github.com/alicevision/AliceVision/pull/1200)
- Reuse image pair io functionality [PR](https://github.com/alicevision/AliceVision/pull/1281)
- Optimize distortion calibration by 1.25x for certain problems [PR](https://github.com/alicevision/AliceVision/pull/1285)
- Replace usage of std::vector as storage of pixel data with image::Image [PR](https://github.com/alicevision/AliceVision/pull/1282)
- (trivial) Improve parallel `ctest` speed by 1.5x by splitting sfm_panorama test into several executables [PR](https://github.com/alicevision/AliceVision/pull/1272)
- (trivial) Extract removePoorlyOverlappingImagePairs() [PR](https://github.com/alicevision/AliceVision/pull/1288)
- Resurrect hdr test [PR](https://github.com/alicevision/AliceVision/pull/1276)
- Optimize voctree build by up to 40 times for certain problems [PR](https://github.com/alicevision/AliceVision/pull/1277)
- (trivial) Cleanup std::pow usages [PR](https://github.com/alicevision/AliceVision/pull/1274)
- Merge mvsData image and pixel classes with what's in image module [PR](https://github.com/alicevision/AliceVision/pull/1257)
- [image] Introduce a function to retrieve ALICEVISION_ROOT [PR](https://github.com/alicevision/AliceVision/pull/1268)
- [imageMatching] bug fix: file format exporter [PR](https://github.com/alicevision/AliceVision/pull/1280)
- [sfmDataIO] Missing fstream include [PR](https://github.com/alicevision/AliceVision/pull/1278)
- (trivial) Extract some code out of pipeline executables to reusable modules [PR](https://github.com/alicevision/AliceVision/pull/1270)
- Fix ceres-solver deprecations [PR](https://github.com/alicevision/AliceVision/pull/1249)
- Consistently use divideRoundUp() for integer division with rounding up [PR](https://github.com/alicevision/AliceVision/pull/1266)
- Don't use deprecated filesystem copy options [PR](https://github.com/alicevision/AliceVision/pull/1264)
- Fix wrong integer division where rounding up was intended [PR](https://github.com/alicevision/AliceVision/pull/1254)
- (trivial) Avoid repeated calls to std::min() or std::max() [PR](https://github.com/alicevision/AliceVision/pull/1253)
- (trivial) Drop code catering to old OpenImageIO versions [PR](https://github.com/alicevision/AliceVision/pull/1251)
- Fix broken openmp atomic usage [PR](https://github.com/alicevision/AliceVision/pull/1234)
- [sfm] Reduce code duplication in sequential sfm triangulation [PR](https://github.com/alicevision/AliceVision/pull/1217)
- [image] Fix deprecated use of oiio computePixelStats() [PR](https://github.com/alicevision/AliceVision/pull/1247)
- [system] Fix deprecated uses of boost::progress_display [PR](https://github.com/alicevision/AliceVision/pull/1248)
- [bugfix] fix getColmapCompatibleViews  [PR](https://github.com/alicevision/AliceVision/pull/1245)
- [sift] Remove dependency on glibc memcpy [PR](https://github.com/alicevision/AliceVision/pull/1231)
- [fuseCut] Pick isnormal() from std namespace [PR](https://github.com/alicevision/AliceVision/pull/1228)
- Make DisplayProgress class thread safe [PR](https://github.com/alicevision/AliceVision/pull/1235)
- Avoid unnecessary string copies [PR](https://github.com/alicevision/AliceVision/pull/1187)
- Always use external CoinUtils, Osi and Clp libraries [PR](https://github.com/alicevision/AliceVision/pull/1237)
- Fixes #1202: Added an available memory check to constrain the number of CPU c… [PR](https://github.com/alicevision/AliceVision/pull/1203)
- [software] Add colmap exporter [PR](https://github.com/alicevision/AliceVision/pull/1184)
- [sift] Fix building on platforms without SSE2 [PR](https://github.com/alicevision/AliceVision/pull/1220)
- Upgrade dependencies in submodules [PR](https://github.com/alicevision/AliceVision/pull/1226)
- Introduce API for displaying progress [PR](https://github.com/alicevision/AliceVision/pull/1211)
- Remove uses of features removed from C++17 standard [PR](https://github.com/alicevision/AliceVision/pull/1221)
- [sfm] Use opengl coordinate system in Alembic & Meshes and add new alignment from Cameras X Axis [PR](https://github.com/alicevision/AliceVision/pull/1030)
- Fix duplicate symbols across executables [PR](https://github.com/alicevision/AliceVision/pull/1209)
- Improve test reproducibility [PR](https://github.com/alicevision/AliceVision/pull/1195)
- Switch to non-deprecated overload of oiio::ImageBufAlgo::make_kernel() [PR](https://github.com/alicevision/AliceVision/pull/1199)
- [multiview] Fix memory leak in resection test [PR](https://github.com/alicevision/AliceVision/pull/1197)
- Fix typo: temporay -> temporary [PR](https://github.com/alicevision/AliceVision/pull/1193)
- Remove uses of fscanf [PR](https://github.com/alicevision/AliceVision/pull/1190)
- [mvsUtils] Remove trivial boost::filesystem wrappers [PR](https://github.com/alicevision/AliceVision/pull/1189)
- Remove using namespace std and add std:: qualifications where needed [PR](https://github.com/alicevision/AliceVision/pull/1185)
- added fstream to fix build issues [PR](https://github.com/alicevision/AliceVision/pull/1178)
- Change focal length in sfmData file formats [PR](https://github.com/alicevision/AliceVision/pull/1098)
- update zlib dependency [PR](https://github.com/alicevision/AliceVision/pull/1166)
- [hdr] new ExposureSetting class [PR](https://github.com/alicevision/AliceVision/pull/1165)
- [software] incrementalSfM: Add option computeStructureColor [PR](https://github.com/alicevision/AliceVision/pull/1151)
- Add new instruction for vs 2022 [PR](https://github.com/alicevision/AliceVision/pull/1152)
- bad computation for stmap output [PR](https://github.com/alicevision/AliceVision/pull/1154)
- [depthMap] Code simplification [PR](https://github.com/alicevision/AliceVision/pull/1130)
- [sfmData] fix uid: avoid conflicts between very close shots [PR](https://github.com/alicevision/AliceVision/pull/1124)
- [io] Alembic: uint/int compatibility [PR](https://github.com/alicevision/AliceVision/pull/1121)
- [camera] fix principal point correction in undistort [PR](https://github.com/alicevision/AliceVision/pull/1115)
- Add support for Canon R5 [PR](https://github.com/alicevision/AliceVision/pull/1111)
- [image] io: use zips compression for exr [PR](https://github.com/alicevision/AliceVision/pull/1110)
- [sfm] loRansac debug for PnP [PR](https://github.com/alicevision/AliceVision/pull/1004)
- [mesh] use file extension to choose the file format [PR](https://github.com/alicevision/AliceVision/pull/1106)
- Optical center relative to the image center [PR](https://github.com/alicevision/AliceVision/pull/1072)
- Update symbolic Bundle Adjustment [PR](https://github.com/alicevision/AliceVision/pull/1060)
- [sfmDataIO] sample scene generation for I/O version check [PR](https://github.com/alicevision/AliceVision/pull/1093)
- [mesh] Replacing custom code for mesh::saveToObj by using assimp library [PR](https://github.com/alicevision/AliceVision/pull/1094)
- [sw] middlebury import [PR](https://github.com/alicevision/AliceVision/pull/1065)
- Add frameId from image path & new function for feature loading [PR](https://github.com/alicevision/AliceVision/pull/1057)
- [sfm] rmse: bug fix if there is no data at all [PR](https://github.com/alicevision/AliceVision/pull/1043)
- New lens distortion calibration software [PR](https://github.com/alicevision/AliceVision/pull/1035)
- GCC11: fix missing <limits> header. [PR](https://github.com/alicevision/AliceVision/pull/1051)
- [software] Some adjustments to export animated camera [PR](https://github.com/alicevision/AliceVision/pull/1047)
- [matching] fix unsigned index issue in guided matching [PR](https://github.com/alicevision/AliceVision/pull/1042)
- [software] New ColorChecker Detection and Correction [PR](https://github.com/alicevision/AliceVision/pull/973)
- [camera] Allow PINHOLE_CAMERA to be chosen as default model [PR](https://github.com/alicevision/AliceVision/pull/1027)
- [feature] dspsift: if image resolution is small, adjust first octave for extraction [PR](https://github.com/alicevision/AliceVision/pull/1026)
- [software] LdrToHdrMerge: more explicit error message [PR](https://github.com/alicevision/AliceVision/pull/1023)
- [panorama] automatic alignment of up vector [PR](https://github.com/alicevision/AliceVision/pull/1021)
- [software] split360: some fixes [PR](https://github.com/alicevision/AliceVision/pull/1006)
- [matching/sfm] IO: do not call "canonical" on non-existing files [PR](https://github.com/alicevision/AliceVision/pull/1007)
- [sfm] in place rotation of point is not allowed in ceres [PR](https://github.com/alicevision/AliceVision/pull/1001)

### Build, CI, Documentation

- [doc] coin libs are required [PR](https://github.com/alicevision/AliceVision/pull/1397)
- [build] Fix building without opencv [PR](https://github.com/alicevision/AliceVision/pull/1393)
- [docker] Use dates to identify versions of prebuild dependencies [PR](https://github.com/alicevision/AliceVision/pull/1390)
- [doc] CONTRIBUTORS: Add Candice Bentejac and Loic Vital [PR](https://github.com/alicevision/AliceVision/pull/1389)
- [doc] add RELEASING.md [PR](https://github.com/alicevision/AliceVision/pull/1387)
- [doc] INSTALL: Update the list of vcpkg packages to install [PR](https://github.com/alicevision/AliceVision/pull/1385)
- [cmake] all-in-one: upgrade dependencies [PR](https://github.com/alicevision/AliceVision/pull/1325)
- [doc] update INSTALL as osi is not internal anymore [PR](https://github.com/alicevision/AliceVision/pull/1363)
- [cmake] cannot have opencv contrib without opencv [PR](https://github.com/alicevision/AliceVision/pull/1352)
- [cmake] fix for `ALICEVISION_HAVE_OPENCV_CONTRIB` [PR](https://github.com/alicevision/AliceVision/pull/1349)
- [image] Fix build failure with Boost 1.81.0 [PR](https://github.com/alicevision/AliceVision/pull/1335)
- [doc] INSTALL: replace geogram broken link [PR](https://github.com/alicevision/AliceVision/pull/1327)
- [cmake] Add support for cuda-12 [PR](https://github.com/alicevision/AliceVision/pull/1324)
- [build] Use Boost's pi constant instead of M_PI in Geometry [PR](https://github.com/alicevision/AliceVision/pull/1322)
- [cmake] propagate dependencies [PR](https://github.com/alicevision/AliceVision/pull/1312)
- [cmake] all-in-one: add boost json [PR](https://github.com/alicevision/AliceVision/pull/1307)
- [cmake][dep] update TBB to oneAPI version [PR](https://github.com/alicevision/AliceVision/pull/1271)
- [docs] Update documentation structure with markdown files [PR](https://github.com/alicevision/AliceVision/pull/1300)
- [cmake][dep] some fixes for openexr [PR](https://github.com/alicevision/AliceVision/pull/1294)
- [cmake][dep] missing iostreams in boost 1.76 [PR](https://github.com/alicevision/AliceVision/pull/1295)
- [cmake][dep] bump zlib to 1.2.13 [PR](https://github.com/alicevision/AliceVision/pull/1292)
- [CMake] Add option to disable usage of modules from OpenCV contrib repo [PR](https://github.com/alicevision/AliceVision/pull/1287)
- [doc] add new contributors [PR](https://github.com/alicevision/AliceVision/pull/1273)
- [doc] INSTALL: remove duplicated information [PR](https://github.com/alicevision/AliceVision/pull/1290)
- [github] Append to vcpkg cache key to expire broken CI cache [PR](https://github.com/alicevision/AliceVision/pull/1269)
- [cmake] Fixes to build on Silicon [PR](https://github.com/alicevision/AliceVision/pull/1241)
- [dep] add PCL as dependency [PR](https://github.com/alicevision/AliceVision/pull/1258)
- [cmake] Add clp dependencies to AliceVisionConfig.cmake.in [PR](https://github.com/alicevision/AliceVision/pull/1262)
- Fix several compile-time warnings [PR](https://github.com/alicevision/AliceVision/pull/1210)
- Remove duplicate description of ALICEVISION_USE_OPENMP in INSTALL.md [PR](https://github.com/alicevision/AliceVision/pull/1230)
- Update vcpkg dependency for Windows [PR](https://github.com/alicevision/AliceVision/pull/1238)
- [doc] Install: update minimal version of mosek [PR](https://github.com/alicevision/AliceVision/pull/1242)
- [build] fixes for recent g++ compiler [PR](https://github.com/alicevision/AliceVision/pull/1239)
- [CMake] Add support for using Eigen alignment with AppleClang [PR](https://github.com/alicevision/AliceVision/pull/1219)
- [ci] do not trigger ci rebuild for .db files [PR](https://github.com/alicevision/AliceVision/pull/1227)
- [CI] Upgrade deps: opencv, expat [PR](https://github.com/alicevision/AliceVision/pull/1224)
- [CMake] Check AV_EIGEN_MEMORY_ALIGNMENT when building dependencies [PR](https://github.com/alicevision/AliceVision/pull/1214)
- [CMake] Add option to build dependencies in parallel [PR](https://github.com/alicevision/AliceVision/pull/1218)
- Add option to compile Eigen with alignment enabled [PR](https://github.com/alicevision/AliceVision/pull/1196)
- [ci] upgrade vcpkg version [PR](https://github.com/alicevision/AliceVision/pull/1179)
- Update actions/stale to latest version v5 [PR](https://github.com/alicevision/AliceVision/pull/1174)
- [cmake] all-in-one: Upgrade dependencies [PR](https://github.com/alicevision/AliceVision/pull/1127)
- Add ca-certificates update to Ubuntu docker build. [PR](https://github.com/alicevision/AliceVision/pull/1118)
- [doc] now aligned with cmake BUILD_SHARED_LIBS [PR](https://github.com/alicevision/AliceVision/pull/1134)
- [doc] fix bibtex [PR](https://github.com/alicevision/AliceVision/pull/1109)
- [doc] readme: update citation [PR](https://github.com/alicevision/AliceVision/pull/1105)
- [ci] fixing a commit id issue in windows github action [PR](https://github.com/alicevision/AliceVision/pull/1089)
- [ci] launch unit tests on windows [PR](https://github.com/alicevision/AliceVision/pull/1087)
- [ci] Add Windows CI on Github Actions [PR](https://github.com/alicevision/AliceVision/pull/1067)
- [doc] install update for windows [PR](https://github.com/alicevision/AliceVision/pull/1079)
- [cmake] Move to C++14 [PR](https://github.com/alicevision/AliceVision/pull/1080)
- [sensorDB] Many updates to the sensor database [List of PRs](https://github.com/alicevision/AliceVision/issues?q=label%3Asensordb+is%3Aclosed+milestone%3A3.0.0)

### Contributors

Thanks to [Fabien Servant](https://github.com/servantftechnicolor), [Gregoire De Lillo](https://github.com/gregoire-dl), [Vincent Demoulin](https://github.com/demoulinv), [Thomas Zorroche](https://github.com/Thomas-Zorroche), [Povilas Kanapickas](https://github.com/p12tic), [Simone Gasparini](https://github.com/simogasp), [Candice Bentejac](https://github.com/cbentejac), [Loic Vital](https://github.com/mugulmd), [Jean Melou](https://github.com/jmelou), [Matthieu Hog](https://github.com/mh0g), [Simon Schuette](https://github.com/natowi), [Ludwig Chieng](https://github.com/ludchieng), [Vincent Scavinner](https://github.com/vscav), [Stella Tan](https://github.com/tanstella) for the major contributions.

All the release contributors:
[a-yonenaga](https://github.com/a-yonenaga), [aidalgol](https://github.com/aidalgol), [AutomatonGeo](https://github.com/AutomatonGeo), [bartoszek](https://github.com/bartoszek), [caiotizio](https://github.com/caiotizio), [camillem](https://github.com/camillem), [canonex](https://github.com/canonex), [chb-jibald](https://github.com/chb-jibald), [Chuardo](https://github.com/Chuardo), [dadul96](https://github.com/dadul96), [DanielDelaporus](https://github.com/DanielDelaporus), [DanielMartin100](https://github.com/DanielMartin100), [Davidsonssilva](https://github.com/Davidsonssilva), [Demarcobank](https://github.com/Demarcobank), [drkoller](https://github.com/drkoller), [ecty99](https://github.com/ecty99), [Ednaordinary](https://github.com/Ednaordinary), [elektrokokke](https://github.com/elektrokokke), [emmanuejtorres](https://github.com/emmanuejtorres), [fabiencastan](https://github.com/fabiencastan), [Garoli](https://github.com/Garoli), [ghost](https://github.com/ghost), [hammady](https://github.com/hammady), [jmenlow](https://github.com/jmenlow), [lapo-luchini](https://github.com/lapo-luchini), [leohumnew](https://github.com/leohumnew), [loqs](https://github.com/loqs), [LRNKN](https://github.com/LRNKN), [object71](https://github.com/object71), [Ogloppi](https://github.com/Ogloppi), [Phoenix-64](https://github.com/Phoenix-64), [PixlEmly](https://github.com/PixlEmly), [remmel](https://github.com/remmel), [rody052](https://github.com/rody052), [sanchayanghosh](https://github.com/sanchayanghosh), [shanji97](https://github.com/shanji97), [SM-26](https://github.com/SM-26), [stellarpower](https://github.com/stellarpower), [techcavy](https://github.com/techcavy), [ThalissonD](https://github.com/ThalissonD), [Tigwin](https://github.com/Tigwin), [Toast-arch](https://github.com/Toast-arch), [Tonycopy](https://github.com/Tonycopy), [tralalafiala](https://github.com/tralalafiala), [tzr250-1kt](https://github.com/tzr250-1kt), [xzuyn](https://github.com/xzuyn)


## Release 2.4.0 (2021/02/26)

### Release Notes Summary

 - [panorama] PanoramaCompositing: new algorithm with tiles to deal with large panoramas [PR](https://github.com/alicevision/AliceVision/pull/947)
 - [feature] Improve robustness of sift features extraction on challenging images: update default values, add new filtering and add dsp-sift variation [PR](https://github.com/alicevision/AliceVision/pull/935)
 - [fuseCut] Reduce mesh artefacts from numerical issues by improving the tetrahedral intersection [PR](https://github.com/alicevision/AliceVision/pull/848) [PR](https://github.com/alicevision/AliceVision/pull/972) [PR](https://github.com/alicevision/AliceVision/pull/984)
 - [fuseCut] Improve mesh quality with a new graphcut post-processing: cells status filtering by solid angle ratio [PR](https://github.com/alicevision/AliceVision/pull/977)
 - [software] Texturing: improve memory estimation [PR](https://github.com/alicevision/AliceVision/pull/982)
 - [panorama] New options to init with known poses [PR](https://github.com/alicevision/AliceVision/pull/964)

### Other Improvements and Bug Fixes

 - [fuseCut] added a parameter to filter points based on number of observations [PR](https://github.com/alicevision/AliceVision/pull/885)
 - [hdr] minor code cleanup in sampling [PR](https://github.com/alicevision/AliceVision/pull/976)
 - [matching] implement cross check for feature matching [PR](https://github.com/alicevision/AliceVision/pull/980)
 - [meshing] Digging using shape from silhouette principle [PR](https://github.com/alicevision/AliceVision/pull/971)
 - [meshing] More control on graph cut post processing [PR](https://github.com/alicevision/AliceVision/pull/987)
 - [software] Add mesh utilities softwares [PR](https://github.com/alicevision/AliceVision/pull/969)
 - [software] MeshFiltering: smoothing and filtering options on subset of the geometry [PR](https://github.com/alicevision/AliceVision/pull/970)
 - [software] sfmTransform: new from_center_camera [PR](https://github.com/alicevision/AliceVision/pull/986)
 - [software] CameraInit: allow to assign the same pose to multiple images [PR](https://github.com/alicevision/AliceVision/pull/951)
 - [software] ExportAnimatedCamera: Export distortion map in exr format [PR](https://github.com/alicevision/AliceVision/pull/963)
 - [depthMap] bug fix on image export if image already computed [PR](https://github.com/alicevision/AliceVision/pull/944)
 - [texturing] segfault fix [PR](https://github.com/alicevision/AliceVision/pull/923)
 - [image] Remove reference to potentially destroyed object [PR](https://github.com/alicevision/AliceVision/pull/937)
 - [sfmData] Support for unavailable folder in sfmData [PR](https://github.com/alicevision/AliceVision/pull/765)
 - Move PRNG initialization out of library [PR](https://github.com/alicevision/AliceVision/pull/934)
 - White balance [PR](https://github.com/alicevision/AliceVision/pull/932)
 - Preliminary work for lighting estimation from normal maps and albedo [PR](https://github.com/alicevision/AliceVision/pull/797)

### Build, CI, Documentation

 - [cmake] Update deps [PR](https://github.com/alicevision/AliceVision/pull/985) [PR](https://github.com/alicevision/AliceVision/pull/978)
 - [cmake] fix sm86 only for cuda >= 11.1 [PR](https://github.com/alicevision/AliceVision/pull/925)
 - [cmake] Use modern cmake to manage OpenGV [PR](https://github.com/alicevision/AliceVision/pull/994)
 - [cmake] update flags for Intel i9-10900K [PR](https://github.com/alicevision/AliceVision/pull/993)
 - [ci] speedup functional tests [PR](https://github.com/alicevision/AliceVision/pull/958)
 - [sensorDB] Many updates to cameraSensors.db


## Release 2.3.1 (2020.10.14)

 - [software] FeatureExtraction: minor update to SIFT memory estimation [PR](https://github.com/alicevision/AliceVision/pull/917)


## Release 2.3.0 (2020.10.09)

### Release Notes Summary

 - [hdr] Improved HDR calibration, including new LdrToHdrSampling for optimal sample selection
[PR](https://github.com/alicevision/AliceVision/pull/835) [PR](https://github.com/alicevision/AliceVision/pull/749) [PR](https://github.com/alicevision/AliceVision/pull/899)
 - [panorama] New Panorama Stitching nodes with support for fisheye lenses
[PR](https://github.com/alicevision/AliceVision/pull/678) [PR](https://github.com/alicevision/AliceVision/pull/749) [PR](https://github.com/alicevision/AliceVision/pull/844) [PR](https://github.com/alicevision/AliceVision/pull/857) [PR](https://github.com/alicevision/AliceVision/pull/871)
 - [texturing] Improve texturing quality on (low-poly) user mesh after retopology [PR](https://github.com/alicevision/AliceVision/pull/677)
 - [multiview] Addition of 10 points relative pose solver [PR](https://github.com/alicevision/AliceVision/pull/474)
 - [sfm] Coordinate system alignment to specific markers or between scenes [PR](https://github.com/alicevision/AliceVision/pull/695)
 - [matching] Add methods in imageMatching [PR](https://github.com/alicevision/AliceVision/pull/730)
 - [sfmData] More generic metadata support [PR](https://github.com/alicevision/AliceVision/pull/820) [PR](https://github.com/alicevision/AliceVision/pull/865)
 - [software] New ImageProcessing software [PR](https://github.com/alicevision/AliceVision/pull/741) [PR](https://github.com/alicevision/AliceVision/pull/763) [PR](https://github.com/alicevision/AliceVision/pull/826) [PR](https://github.com/alicevision/AliceVision/pull/825)

### Other Improvements and Bug Fixes

 - [cameraInit] Support more extensions and add new viewId generation method [PR](https://github.com/alicevision/AliceVision/pull/823)
 - [depthMap] Fixed potential index out of range exception in PlaneSweepingCuda [PR](https://github.com/alicevision/AliceVision/pull/694) 
 - [feature] Fix popsift static release [PR](https://github.com/alicevision/AliceVision/pull/807)
 - [fuseCut] Improve delaunayGraphCut [PR](https://github.com/alicevision/AliceVision/pull/832) 
 - [fuseCut] Similarity map is optional [PR](https://github.com/alicevision/AliceVision/pull/831) 
 - [hdr] Update compensation of aperture and ISO change in HDR fusion [PR](https://github.com/alicevision/AliceVision/pull/863)
 - [image] Fix bug with input RGBA Images [PR](https://github.com/alicevision/AliceVision/pull/743)
 - [keyframe] KeyframeSelector: add padding in created filenames [PR](https://github.com/alicevision/AliceVision/pull/740)
 - [matching] Fix unmatching declaration and definitions [PR](https://github.com/alicevision/AliceVision/pull/810)
 - [meshing] Fix load of depth maps without metadata [PR](https://github.com/alicevision/AliceVision/pull/738)
 - [meshing] Simplify input depth map folders [PR](https://github.com/alicevision/AliceVision/pull/815)
 - [panorama] HDR Panorama: Handle very high intensities [PR](https://github.com/alicevision/AliceVision/pull/891)
 - [panorama] Panorama: Add Graphcut in compositing [PR](https://github.com/alicevision/AliceVision/pull/850) [PR](https://github.com/alicevision/AliceVision/pull/876)
 - [sfm] Add scale as parameter in functions for sfm [PR](https://github.com/alicevision/AliceVision/pull/736)
 - [sfm] Bugfix: ceres::AngleAxisRotatePoint cannot be used in-place [PR](https://github.com/alicevision/AliceVision/pull/696)
 - [sfm] Performance improvements: optimize RemoveOutliers_AngleError [PR](https://github.com/alicevision/AliceVision/pull/841)
 - [sfm] SfM statistics [PR](https://github.com/alicevision/AliceVision/pull/774)
 - [sfmDataIO] Add Observation scale in IO [PR](https://github.com/alicevision/AliceVision/pull/766)
 - [sfmSfM] Major bug fix on BlockOrder in BACeres [PR](https://github.com/alicevision/AliceVision/pull/849)
 - [software] ConvertSfMFormat: bug fix: remove pose only if it exists [PR](https://github.com/alicevision/AliceVision/pull/859)
 - [software] Image matching: fix min nb images [PR](https://github.com/alicevision/AliceVision/pull/739) 
 - [software] ImageProcessing: add option to fix non finite pixels [PR](https://github.com/alicevision/AliceVision/pull/890)
 - [software] ImageProcessing: add storageDataType option [PR](https://github.com/alicevision/AliceVision/pull/896)
 - [software] PanoramaCompositing: option to select the percentage of upscaled pixels:[PR](https://github.com/alicevision/AliceVision/pull/864)
 - [software] PanoramaEstimation: sort input images by shooting time to select the center camera [PR](https://github.com/alicevision/AliceVision/pull/868)
 - [software] PanoramaInit: Add an extra image rotation to each camera declared the input xml [PR](https://github.com/alicevision/AliceVision/pull/867)
 - [software] PanoramaInit: Add new restriction to circle detection [PR](https://github.com/alicevision/AliceVision/pull/872)
 - [software] PanoramaInit: improve fisheye circle detection[PR](https://github.com/alicevision/AliceVision/pull/897)
 - [software] SfMTransfer: New option to copy intrinsics from one sfmdata to another [PR](https://github.com/alicevision/AliceVision/pull/882)
 - [tracks] Fix tracks filtering [PR](https://github.com/alicevision/AliceVision/pull/761)
 - [voctree] Fix possible overflow in unsigned subtraction [PR](https://github.com/alicevision/AliceVision/pull/786)
 - [windows] Add main function wrapper to catch exceptions and avoid annoying error popup on windows [PR](https://github.com/alicevision/AliceVision/pull/768)
 - [sensorDB] Many updates to cameraSensors.db

### Build, CI, Documentation

 - [matching] Build fix for MSVC 2019 [PR](https://github.com/alicevision/AliceVision/pull/805)
 - [build] Avoid circular dependency [PR](https://github.com/alicevision/AliceVision/pull/839)
 - [build] Fix build against cuda<=11 [PR](https://github.com/alicevision/AliceVision/pull/851)
 - [build] Fix offset logging against boost 1.71 [PR](https://github.com/alicevision/AliceVision/pull/709)
 - [build] Fix prettyprint MSVC compatibility [PR](https://github.com/alicevision/AliceVision/pull/784)
 - [build] Fix unit test includes [PR](https://github.com/alicevision/AliceVision/pull/773)  
 - [build] Make sure suitesparse build with jenkins pipeline [PR](https://github.com/alicevision/AliceVision/pull/801)
 - [build] Remove unused embedded cxsparse [PR](https://github.com/alicevision/AliceVision/pull/770)
 - [build] Workaround for a build incompatibility between prettyprint and eigen [PR](https://github.com/alicevision/AliceVision/pull/705) 
 - [ci] Add github actions CI [PR](https://github.com/alicevision/AliceVision/pull/782)
 - [ci] AppVeyor: Build with VS 2019, Cuda 11 [PR](https://github.com/alicevision/AliceVision/pull/776) 
 - [ci] Build alicevision snapshot binaries (win) [PR](https://github.com/alicevision/AliceVision/pull/753)
 - [ci] remove travis [PR](https://github.com/alicevision/AliceVision/pull/843) 
 - [cmake] All-in-one: update geogram to 1.7.3 [PR](https://github.com/alicevision/AliceVision/pull/732) 
 - [cmake] Build all-in-one: add gmp, mpfr, ffmpeg [PR](https://github.com/alicevision/AliceVision/pull/892)
 - [cmake] Fix eigen flags [PR](https://github.com/alicevision/AliceVision/pull/791)
 - [cmake] Fix findOpenEXR missing  escapes [PR](https://github.com/alicevision/AliceVision/pull/788) 
 - [cmake] Missing Boost::boost dependency [PR](https://github.com/alicevision/AliceVision/pull/771)
 - [cmake] Update required CMake version [PR](https://github.com/alicevision/AliceVision/pull/811) 
 - [cmake] Using modern cmake for boost [PR](https://github.com/alicevision/AliceVision/pull/719)
 - [cmake] Add cuda support for SM 86 [PR](https://github.com/alicevision/AliceVision/pull/911)
 - [doc] Build: add notice that english language pack is required for VS/vcpkg [PR](https://github.com/alicevision/AliceVision/pull/804)
 - [doc] Fixed citation [PR](https://github.com/alicevision/AliceVision/pull/716)
 - [doc] INSTALL.md: add boost-timer and lz4 to vcpkg install command [PR](https://github.com/alicevision/AliceVision/pull/704)
 - [docker] Clean docker images for centos and ubuntu [PR](https://github.com/alicevision/AliceVision/pull/881)
 - [docker] Fix ubuntu dockerfile [PR](https://github.com/alicevision/AliceVision/pull/800)
 - [docker] Fixes docker and all-in-one cmake build [PR](https://github.com/alicevision/AliceVision/pull/785)
 - [docker] Minor Changes in the docker build command [PR](https://github.com/alicevision/AliceVision/pull/760)
 - [docker] Release docker [PR](https://github.com/alicevision/AliceVision/pull/762) 
 - [github] Added actions for stale issues [PR](https://github.com/alicevision/AliceVision/pull/693)
 - [github] Fix stale.yml [PR](https://github.com/alicevision/AliceVision/pull/781)
 - [meshing] Code refactoring [PR](https://github.com/alicevision/AliceVision/pull/667)
 - [sensor] Replace deprecated basestring with str (Py3) [PR](https://github.com/alicevision/AliceVision/pull/722)
 - [sfmDataIO] Throw if we try to load an abc file and AliceVision is built without Alembic support [PR](https://github.com/alicevision/AliceVision/pull/878)
 - [system] Logger levels were not case insensitive [PR](https://github.com/alicevision/AliceVision/pull/711)
 - [windows] Fixing vs2019 build options [PR](https://github.com/alicevision/AliceVision/pull/796)

For more details see all PR merged: https://github.com/alicevision/AliceVision/milestone/31


## Release 2.2.0 (2019.08.08)

Release Notes Summary:

 - Texturing: Largely improve the Texturing quality using Multi-Band Blending technique. [PR](https://github.com/alicevision/AliceVision/pull/629)
 - Texturing: Support for RAW image files: Libraw update and fix its usage in OpenImageIO (fix color shift and use correct gamma). [PR](https://github.com/alicevision/AliceVision/pull/645)
 - Texturing: Performance optimization by iterating over multiple output textures at the same time. [PR](https://github.com/alicevision/AliceVision/pull/615)
 - Texturing: Add support for UDIM in UV mapping and use it by default. [PR](https://github.com/alicevision/AliceVision/pull/596)
 - Meshing: Export the dense point cloud in Alembic and computes the points colors [PR](https://github.com/alicevision/AliceVision/pull/597).
 - Meshing: New option to export the full raw dense point cloud in the Meshing software (with all 3D points candidates before cut and filtering). [PR](https://github.com/alicevision/AliceVision/pull/597)
 - Meshing: Adds an option to export color data per vertex and MeshFiltering correctly preserves colors. [PR](https://github.com/alicevision/AliceVision/pull/661).
 - HDR: New HDR module for the fusion of multiple LDR images with the corresponding new software LDR2HDR. It implements linear fusion as well as Camera Response Function (CRF) calibration with Robertson, Debevec or Grossberg methods. [PR](https://github.com/alicevision/AliceVision/pull/613)
 - PrepareDenseScene: Add experimental option to correct Exposure Values (EV) of input images to uniformize dataset exposures. [PR](https://github.com/alicevision/AliceVision/pull/652)
 - FeatureExtraction: Include CCTag in the release binaries both on Linux and Windows. [PR](https://github.com/alicevision/AliceVision/pull/657)

Full Release Notes:

 - DepthMap: Bug fix if no nearby cameras (a big bottleneck of the release 2019.1). [PR](https://github.com/alicevision/AliceVision/pull/616)
 - DepthMap: Bug fix: missing allocation when reducing the number of planes. [PR](https://github.com/alicevision/AliceVision/pull/642)
 - DepthMap: Bug fix: SGM early stop condition could provide an unallocated buffer to the refine step [PR](https://github.com/alicevision/AliceVision/pull/671)
 - FeatureExtraction: Bug fix in AKAZE filtering (sort keypoints by size before grid filtering). [PR](https://github.com/alicevision/AliceVision/pull/635)
 - FeatureMatching: Bug fix: use ranges prefix to avoid overwrites of matching files when using ImageMatching. [PR](https://github.com/alicevision/AliceVision/pull/628)
 - SfM: Improve SfM colorization performances. [PR](https://github.com/alicevision/AliceVision/pull/597)
 - SfM: Fix intrinsics edges management in the Local Bundle Adjustment (LBA). [PR](https://github.com/alicevision/AliceVision/pull/624)
 - Texturing: UVAtlas: downscale charts to fit in texture to avoid bug with small "Texture Side". Apply downscale factor to fit larger charts into the texture, and use it when filling atlases and computing final UV textures. [PR](https://github.com/alicevision/AliceVision/pull/598)
 - SfM: Fix management of intrinsics related connections in the local bundle adjustment graph which could lead to removal of wrong edges and potential corruption of the graph. This could end up with the SfM going in an infinite loop and/or undefined behavior. [PR](https://github.com/alicevision/AliceVision/pull/624)
 - DepthMap: Remove useless remaining image transpositions. [PR](https://github.com/alicevision/AliceVision/pull/653)
 - CCTag: Major update to get Windows compatibility. [PR](https://github.com/alicevision/CCTag/pull/78)
 - SfMDataIO: Change root nodes (XForms instead of untyped objects) of Alembic SfMData for better interoperability with other 3D graphics applications (in particular Blender and Houdini). [PR](https://github.com/alicevision/AliceVision/pull/659)
 - DepthMap: Add option to create Normal Maps in DepthMapEstimation (not used in other part of the pipeline for now). [PR](https://github.com/alicevision/AliceVision/pull/604)
 - DepthMap: Add option to export similarity volumes in Alembic files (for research analysis). [PR](https://github.com/alicevision/AliceVision/pull/603)
 - DepthMap: remove useless images transpositions. [PR](https://github.com/alicevision/AliceVision/pull/653), [and fix](https://github.com/alicevision/AliceVision/pull/666)
 - Texturing: Add option to choose the internal colorspace used for color fusion. [PR](https://github.com/alicevision/AliceVision/pull/651)
 - Texturing: Add option to correct exposure values during Texturing. [PR](https://github.com/alicevision/AliceVision/pull/656)
 - FeatureExtraction: include CCTag in the release binaries. [PR](https://github.com/alicevision/AliceVision/pull/657).
 - Travis: Update libraries dependencies. [PR](https://github.com/alicevision/AliceVision/pull/637)
 - SensorDB: Add more than 100 new models. [PR](https://github.com/alicevision/AliceVision/pull/618)
 - CMake: New doxygen documentation target. [PR](https://github.com/alicevision/AliceVision/pull/627)
 - CMake: Improve usage as third-party library. [PR](https://github.com/alicevision/AliceVision/pull/672)

For more details see all PR merged: https://github.com/alicevision/AliceVision/milestone/30


## Release 2.1.0 (2019.01.30)

Release Notes Summary:
 - More complete sensor database and better matching as well as explicit status for lens initialization
 - Add support for rig of cameras. This enables to add more contraints if you make acquisition with multiple synchronized devices.
 - Support for reconstruction with projected light patterns and texturing with another images set.
 - Better estimation of the space to reconstruct to limit the reconstruction zone. This avoid to reconstruct low quality and useless areas around the main object/environment.
 - New option to directly mesh the SfM results. This provides a quick solution to get a draft mesh (without cuda requirement).
 - Reduce IO and intermediate files in MVS part of the pipeline.

Full Release Notes:
 - sensorDB: Update the sensors database with much more models
 - sensorDB: Use `FocalLengthIn35mmFilm` metadata as a fallback if available
 - sensorDB: Improve matching of metadata make/model with sensor database
 - CameraInit: New management of intrinsics (serial number, error/warning log) with an explicit InstrinsicInitMode enum
 - CameraInit: Fix intrinsics grouping with serial number as expected and add metadata "AliceVision::SensorWidth"
               (or "AliceVision::SensorWidthEstimated" if estimated from `FocalLength`/`FocalLengthIn35mmFilm`)
 - CameraInit: Explicit warning if serial number is missing
 - SfM: Add support for rig of cameras. This information is used as a new constraint in the SfM. This option can now be combined with localBA.
        You need to use a specific folder hierarchy in the input images files (for instance: “/my/dataset/rig/0/DSLR_0001.JPG”, “/my/dataset/rig/1/DSLR_0001.JPG”) to provide this information.
 - PrepareDenseScene: New `imagesFolders` option to override input images. This enables to use images with light patterns projected for SfM and MVS parts
                      and do the Texturing with another set of images.
 - Meshing fusion: New option to use SfM landmarks (mesh from sparse point cloud) or combine them with depth maps
 - Meshing: Add option `estimateSpaceFromSfM` to better estimate the bounding box of the reconstruction and avoid useless reconstruction of the environment
 - MVS: Reduce IO and intermediate files. Use sfmData in the MVS pipeline (instead of MVS seeds, ini file) and do not export depthmap files before refine
 - ExportAnimatedCamera: Improved solution to export an SfM result into an animated camera
 - PrepareDenseScene: Remove hackish software `CameraConnection` (now properly done in PrepareDenseScene)
 - PrepareDenseScene: Allow parallelization on renderfarm
 - FeatureExtraction: Add grid filtering in AKAZE (to avoid unmanageable number of keypoints on well textured images)
 - DepthMap: Fix memory leak in SGMVolume
 - Meshing: Fix loading of OBJ with normals
 - System: Improve gpu detection and logging
 - DepthMap: Enable usage of multiple GPUs by default with an option to add a limit
 - DepthMap: Fuse estimate & refine in a single loop (fuse “computeDepthMapsPSSGM” with “refineDepthMaps”)  and use the same image cache
 - DepthMap: Remove depthMapInfo files and use image metadata instead
 - DepthMap: Initial refactoring for better readability to prepare the optimization work
 - SfM: Refactoring of localBA (now fused with the standard BA code to avoid redundancy)
 - ConvertSfMFormat: Users can now specify a view whitelist to filter views
 - SfM: The user can provide only one image of the initial pair and it will choose the 2nd image automatically.
        This allows to ensure that the reconstruction start from a specific part of the scene without choosing the image pair manually.
 - SfMTransform: Allow to choose one view as the origin for the coordinate system
 - SfM Augmentation from known poses: When the sfm starts with existing cameras poses but without landmarks, it now does the triangulation first.
 - LightingEstimation: New module with preliminary low-level methods for lighting estimation (spherical harmonics)
 - MeshDenoising: Fix ignored parameter denoising outer iteration
 - Meshing: Bug fix (infinity loop) in Meshing when the BBox is empty
 - SfM LocalBA: reduce minimum number of images to use sparse BA (change threshold from 100 to 20) to speedup large reconstructions when localBA is used.
 - Minor build fix for compatibility with ceres 2.


## Release 2.0.0 (2018.08.09)

Release of the full 3D reconstruction pipeline.

### 2018.07

 - New Docker images for Centos 7 and Ubuntu 18
 - New "make bundle" for packaging
 - Refactor split sfm / sfmData / sfmDataIO
 - New visibility remapping methods: Push, PullPush
 - Improve texturing quality with better image selection
 - SfM support multiple folders for features and matches
 - PopSiftImageDescriber: no initialization if not used
 - Offline camera tracking improvements
 - Export animated camera ABC
 - Export undistorted images and filter option
 - MeshroomMaya script integration Image Plane
 - Minor fixes in cameraSensors DB search
 - New fallback if no sensor width info available but FocalLengthIn35mmFilm metadata is present
 - ImageMatchingMultiSfM: add “a_a+a_b” option

### 2018.06

 - SfM Augmentation: lock cameras from the initial reconstruction
 - SfM: Add option in order to disable the cleaning of tracks forks

### 2018.03

 - Merge the MVS pipeline in the main branch
 - New options for better auto UVs based on geogram (needs reasonable mesh size in input)
 - Use full resolution images in the MVS pipeline: PrepareDenseScene creates full resolution undistorted images, DepthMap computes downscale when loading images and Texturing can be done in full resolution.
 - New depth map fusion with a multi-scale approach (using nanoflann)
 - ImageMatching: Fix conflict if multiple images with the same UID
 - Add SIFT_UPRIGHT as an ImageDescriber

## Release 1.0.0 (2018.03.07)

Release of the Structure-From-Motion pipeline.

### 2018.02

 - Support Raw and Exr input files
 - Texturing: add multithreading / clean iteration over pixels in triangle
 - MVS: use UIDs
 - Major MVS refactoring
 - New Mesh Denoiser and Decimate based on MeshSDFilter
 - Integration of Uncertainty computation step

### 2018.01

 - Meshing: Remove facets with helper points but limit holes creation
 - Texturing: Don’t modify the topology
 - Meshing: Add an option to keep only the largest facets group
 - cmake: Geogram as a submodule of cmpmvs
 - Update SfM
 - Modify Image Matching
 - SfM Reorientation software
 - Use OpenMP for featureExtraction with a new imageDescriber memory needs estimation
 - Rewrite “Identify the track to triangulate” in triangulateMultiViews_LORANSAC
 - popSIFT directly on floating point images
 - Use relative path for features and matches in SfM
 - Remove cereal dependency
 - Remove static functions in headers
 - MVS: Add namespace per module
 - MVS: Build as dynamic libraries
 - MVS: Remove unneeded intermediate images

### 2017.12

 - Reduce the amount of storage for intermediate files and improve robustness to kill/restart jobs on renderfarm
 - New software to create simplified versions of the mesh
 - Use OpenImageIO in MVS
 - Use floating point image in texturing

### 2017.11

 - New Local Bundle Adjustment to speedup SfM on large scenes
 - Retexturing on an external user mesh with a retopology (no visibility information and support user UVs) with a first visibilities remapping method.
 - Add new images to a previous SfM reconstruction
 - Use OpenImageIO in SfM

### 2017.10

 - Reduce memory usage on Meshing

### 2017.10

 - SfM: Support for RIG of synchronized cameras/DSLR with a new constraint between rigidly fixed cameras
 - New software utility for 360° cameras

### 2017.08

 - Tetrahedralization scoring with boost maxflow

### 2017.07

 - Meshing tetrahedralization with geogram
 - Texturing speedup
 - Rewrite CUDA layer

### 2017.06

 - SfM: Weighting on image describers to combine them correctly

### 2017.03

 - MVS: Support for multiple image resolutions

### 2017.02

 - MVS: Code comments and documentation
 - MVS: Performance improvements
 - Texturing: Fix UV coords
 - MVS: Split Meshing and Texturing steps
 - Texturing: Rewrite edge padding for performance reasons

### 2017.01

 - MVS: Linux code porting

### 2016

 - Integration of PopSift: a new GPU SIFT implementation
 - SfM: Add LoRansac
 - SfM: New next best view strategy to promote good repartition in images. Same rule in image pair selection.
 - SfM: Optional filtering of the input tracks with a minimal track length
 - SfM: Optional limitation on the number of input matches from an image pair
 - Sort features and matches
 - FeatureExtraction: Limit the number of features per image with grid filtering to ensure good repartition
 - New software to align reconstructions based on common cameras (using UID)

### 2015

 - New Alembic file format to store sparse point cloud and cameras
 - Integration of new CCTag markers with CPU and CPU implementations
 - SfM use UID
 - Support for cameras RIG calibration and localization based on opengv
 - Camera lens calibration based on opencv
 - New camera localization module
 - SfM speedup by precomputing tracks visibilities and adjusting BA thresholds
 - New Image Matching based on vocabulary tree approach
 - Features extraction and features matching parallelization on multiple computers for renderfarm usage

### 2014

 - First public source code release of the SfM pipeline
