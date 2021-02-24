# AliceVision Changelog

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
