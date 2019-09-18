# AliceVision Changelog

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
