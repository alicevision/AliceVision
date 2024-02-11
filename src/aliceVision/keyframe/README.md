# Keyframe Selection

This module provides several methods to perform a keyframe selection.

The goal of the keyframe selection is to extract, from an input video, an input sequence of images or an SfMData file, keyframes.
Two methods are currently supported:
- a **regular** selection method, which selects keyframes regularly across the input video / sequence / SfMData according to a set of parameters;
- a **smart** selection method, which analyses the sharpness and motion of all the frames to select those which are deemed the most relevant (a frame is considered relevant if it contains significant motion in comparison to the last selected keyframe while being as sharp as possible).

The selected keyframes can be written as JPG, PNG or EXR images, and the storage data type can be specified when the EXR file extension is selected. The selected keyframes will not be written on disk if the selected output extension is `NONE`.

The keyframe selection module supports the following inputs:
- a path to a video file (e.g. "/path/to/video.mp4")
- a path to a folder containing images (e.g. "/path/to/folder/")
- a path to a folder containing images with a regular expression (e.g. "/path/to/folder/*.exr")
- a path to an SfMData file (e.g. "/path/to/sfmData.sfm")

Camera rigs are also supported for all the inputs except the SfMData file.

In addition to writing the selected keyframes on disk, two SfMData files are written:
- one that contains all the selected keyframes
- one that contains all the frames that were not selected as keyframes

_N.B: If the input is a video file, the SfMData file which contains the rejected frames will not be written at all, since none of these frames is available on disk. As the selected keyframes will be written at the end of the selection, the SfMData file containing the keyframes **will** be written. However, if the output extension is set to `NONE`, no SfMData file will be written as even the keyframes will not be available on disk._

## Regular selection method

The regular selection samples frames regularly over time with respect to some user-provided constraints, that can be combined:
- `minFrameStep`: the minimum number of frames between two selected keyframes. If only `minFrameStep` is set, one keyframe will be selected every `minFrameStep` all along the video.
- `maxNbOutFrames`: the maximum number of selected keyframes (if set to 0, the number of selected keyframes will be unlimited). If only `maxNbOutFrames` is set, `maxNbOutFrames` keyframes equally spaced along the video will be selected.

If both `minFrameStep` and `maxNbOutFrames` are set, up to `maxNbOutFrames` keyframes separated by at least `minFrameStep` frames will be selected. Examples of the parameter combinations are available in the [Examples](#examples) section.

### Advanced regular selection

For a more advanced regular selection, another parameter, `maxFrameStep`, is available to combine a relatively strict sampling with a maximum number of output frames. `maxFrameStep` sets the maximum number of frames between two selected keyframes and ensures that there will not be way more frames between two keyframes than expected when `maxNbOutFrames` is also set. `maxFrameStep` always takes precedence over `maxNbOutFrames`, meaning that the input video / sequence might not be sampled entirely for all the constraints to be respected.

Combinations of the different parameters and the influence of `maxFrameStep` are shown in the [Examples](#examples) section.


### Examples

The expected behaviour for the regular selection depending on the set parameters can be summed up as follows:

- If only `minFrameStep` is set, the whole sequence will be sampled and a keyframe will be selected every `minFrameStep`. E.g: if a sequence has 2000 frames and `minFrameStep = 100`, 21 keyframes will be selected, with exactly 100 frames between them.

- If `minFrameStep` and `maxNbOutFrames` are set, there will never be less than `minFrameStep` between the keyframes, but there might be more for the whole sequence to be sampled while respecting `maxNbOutFrames`. E.g: if a sequence has 2000 frames, `minFrameStep = 100` and `maxNbOutFrames = 10`, 10 keyframes with 222 frames between them will be selected, so both `maxNbOutFrames` and `minFrameStep` are respected. If the sequence has 500 frames and `minFrameStep = 100` / `maxNbOutFrames = 10`, there will be 6 keyframes with 100 frames between them. No matter the value of the parameters, the entire sequence will be sampled.

- If `minFrameStep` and `maxFrameStep` are both set but `maxNbOutFrames` is not, then the step between two keyframes will be exactly between `minFrameStep` and `maxFrameStep`. If `minFrameStep = 100` and `maxFrameStep = 200` without other constraints, it is equivalent to setting `minFrameStep = 150`.

- If `minFrameStep`, `maxFrameStep` and `maxNbOutFrames` are all set, then `maxFrameStep` prevents the step between two frames from increasing too much to respect `maxNbOutFrames`. With the sequence of 2000 frames, having `minFrameStep = 100`, `maxFrameStep = 150` and `maxNbOutFrames = 10` will lead to 10 keyframes with 150 frames between each, and the sampling will stop before reaching the end of the sequence so that all the constraints are respected. In the same example, if `maxFrameStep = 300`, then there will be 10 keyframes with 222 frames between them, and the whole sequence will be sampled.

## Smart selection method

The smart selection works in two steps:
- for each frame in the input video / sequence, a sharpness score and a motion score are computed;
- the sharpness and motion scores are used as well as the temporal position of the evaluated frame to determine whether the frame will be selected.

The method aims at selecting a frame that is as sharp as possible with significant motion compared to the previously selected frame: consecutive frames should not be selected as keyframes if they do not contain enough motion, even if they are both very sharp.

The minimum and maximum number of selected keyframes with the smart method can be set with the following parameters:
- `minNbOutFrames`: the minimum number of selected keyframes;
- `maxNbOutFrames`: the maximum number of selected keyframes.

Masks (e.g. segmentation masks) can also be provided with the `maskPaths` parameters. If they are provided and valid (i.e. the number of masks and their size is identical to the input frames'), they are loaded alongside the input frames and are applied during the computation of the scores.

### Frame scoring

For both the sharpness and motion scores, the evaluated frame is converted to a grayscale OpenCV matrix that may be rescaled
Scores are computed on grayscale images, which may have been rescaled using the `rescaledWidth` parameter.

#### Sharpness score

The Laplacian of the input frame is first computed, followed by the integral image of the Laplacian. A sliding window of size `sharpnessWindowSize` is used to compute the standard deviation of the averaged Laplacian locally. The final sharpness score will be the highest standard deviation found.

The image is evaluated with a sliding window instead of as a whole to prevent giving a bad score (low standard deviation) to a frame that contains a sharp element but is overall blurry.

If a valid mask has been provided, it is applied on the integral image of the Laplacian. All masked pixels are then set to 0, and they are excluded from the standard deviation computation. If the number of masked pixels within the window exceeds 50%, then the standard deviation for that specific sliding window position is skipped, and the window is moved to the next position. 

#### Motion score

The dense optical flow of a frame is computed. The frame is then divided into cells of `flowCellSize` pixels in which the motion vectors are averaged to obtain a displacement value (in pixels) within that cell. Once all the displacement values have been computed, the median value of these displacement values is used as the motion score.

If a valid mask has been provided, it is applied to the computed motion vectors, and the masked motion vectors are excluded from the displacement computation. If the mask covers at least 50% of the cell that is evaluated, the computation for that cell is skipped altogether: it will be ignored when the median value of the displacement scores is computed.

### Selection

Once both the sharpness and motion scores have been computed, subsequences are identified based on the motion accumulation across frames. The motion accumulation threshold is set with `pxDisplacement` which represents, in per cent, the number of pixels that need to have moved since the last keyframe for the motion to be significant. As the motion scores represent a displacement value for each frame, summing them over time until the accumulation reaches the threshold allows to divide the input video / sequence into subsequences that all contain significant motion.

Within each subsequence, a single frame is to be selected as a keyframe. Before proceeding to the selection itself, the number of identified subsequences is checked to ensure that the minimum and maximum number of requested output keyframes are respected.
- If not enough subsequences have been identified, the motion accumulation threshold is lowered iteratively with a step of 0.5 px until it either reaches 0 or gives out an expected number of subsequences. If 0 is reached, the motion accumulation criterion stops making sense and is thus replaced by a regular sampling: in that specific case, the smart method falls back to the regular method's behaviour. 
- If too many subsequences have been identified, the motion accumulation threshold is increased iteratively with a step of 0.5 px until an acceptable number of subsequences is identified.

A keyframe is thus selected for each subsequence, based on its sharpness score as well as its position in its subsequence: the sharpness score of each frame is combined to a weight based on its position within the subsequence, with the best weights applied to the frames located at the middle of the subsequence, and the worst weights applied to the frames located on the subsequence's borders.

The weights aim at favouring the selection of keyframes that are as temporally far from each other as possible. Using only the sharpness scores to select a keyframe within a subsequence could lead to two consecutive very sharp frames, respectively located at the very end of a subsequence and at the very beginning of the following subsequence, being selected. This would hinder the relevancy of the whole process, as they would likely not contain any significant difference. 

### Debug options

Debug options specific to the smart selection method are available:
- Export scores to CSV: the sharpness and motion scores for all the frames are written to a CSV file;
- Visualise the optical flow: the computed motion vectors are, for each frame, visualised with HSV images that are written as PNG images;
- Skip the sharpess score computations: the motion scores are computed normally, but all the sharpness score computations are skipped and replaced by a fixed value (1.0), which allows to assess the impact of the sharpness score computations (and, by extension, of the motion scores) on the global processing time;
- Skip the frame selection: the scores are computed normally (the sharpness scores can be skipped) but will not be used to perform the final selection. This is mainly useful to determine the processing time solely dedicated to the score computations or, combined with the CSV export export, to evaluate the quality of the scoring without needing to go through the complete selection process.


## API 

- Constructor
```cpp
KeyframeSelector(const std::vector<std::string>& mediaPaths,
                 const std::vector<std::string>& maskPaths,
                 const std::string& sensorDbPath,
                 const std::string& outputFolder,
                 const std::string& outputSfmKeyframes,
                 const std::string& outputSfmFrames);
```
- Selection with regular method
```cpp
void processRegular();
```
- Selection with smart method
```cpp
void processSmart(const float pxDisplacement,
                  const std::size_t rescaledWidthSharpness,
                  const std::size_t rescaledWidthFlow,
                  const std::size_t sharpnessWindowSize,
                  const std::size_t flowCellSize,
                  const bool skipSharpnessComputation = false);
```
- Score computation
```cpp
bool computeScores(const std::size_t rescaledWidthSharpness,
                   const std::size_t rescaledWidthFlow,
                   const std::size_t sharpnessWindowSize,
                   const std::size_t flowCellSize,
                   const bool skipSharpnessComputation);
```
- Write selected keyframes
```cpp
bool writeSelection(const std::vector<std::string>& brands,
                    const std::vector<std::string>& models,
                    const std::vector<float>& mmFocals,
                    const std::string& outputExtension,
                    const image::EStorageDataType storageDataType = image::EStorageDataType::Undefined) const;
```
- Debug options
```cpp
bool exportScoresToFile(const std::string& filename,
                        const bool exportSelectedFrames = false) const;

bool exportFlowVisualisation(const std::size_t rescaledWidth);
```