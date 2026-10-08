{
    "header": {
        "releaseVersion": "2026.1.0+develop",
        "fileVersion": "2.1",
        "nodesVersions": {
            "ApplyCalibration": "1.0",
            "CameraInit": "12.1",
            "ConvertDistortion": "1.0",
            "ConvertSfMFormat": "2.0",
            "CopyFiles": "1.3",
            "ExportAlembic": "1.0",
            "ExportDistortion": "2.0",
            "ExportImages": "1.1",
            "FeatureExtraction": "1.3",
            "FeatureMatching": "2.1",
            "ImageMatching": "2.0",
            "ImageMatchingMultiSfM": "1.1",
            "ImageSegmentationSam3": "2.0",
            "InputFile": "1.0",
            "IntrinsicsTransforming": "1.1",
            "KeyframeSelection": "5.0",
            "MaskEroding": "1.0",
            "MaskProcessing": "1.0",
            "ScenePreview": "2.0",
            "SfMBootStrapping": "4.2",
            "SfMColorizing": "1.0",
            "SfMExpanding": "2.6",
            "SfMFilter": "2.0",
            "SfMPoseInjecting": "1.0",
            "SfMSplitReconstructed": "1.0",
            "SfMSurveyInjecting": "1.1",
            "SfMTransfer": "2.1",
            "TracksBuilding": "1.0",
            "TracksMerging": "3.0",
            "VideoSegmentationSam3": "2.0"
        },
        "template": true
    },
    "graph": {
        "ApplyCalibration_1": {
            "nodeType": "ApplyCalibration",
            "position": [
                -1208,
                10
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "useJson": true
            },
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "Backdrop_1": {
            "nodeType": "Backdrop",
            "position": [
                1746,
                -616
            ],
            "internalInputs": {
                "label": "Previsualisation Keyframes",
                "nodeWidth": 1183,
                "nodeHeight": 222
            }
        },
        "Backdrop_2": {
            "nodeType": "Backdrop",
            "position": [
                4028,
                -81
            ],
            "internalInputs": {
                "label": "Previz final",
                "nodeHeight": 200
            }
        },
        "Backdrop_3": {
            "nodeType": "Backdrop",
            "position": [
                -1157,
                242
            ],
            "internalInputs": {
                "label": "Segmentation",
                "nodeWidth": 646,
                "nodeHeight": 279
            }
        },
        "Backdrop_4": {
            "nodeType": "Backdrop",
            "position": [
                -1257,
                -124
            ],
            "internalInputs": {
                "label": "External Info",
                "nodeWidth": 1096,
                "nodeHeight": 258
            }
        },
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -1577,
                17
            ],
            "inputs": {
                "isSequence": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ConvertDistortion_1": {
            "nodeType": "ConvertDistortion",
            "position": [
                3747,
                384
            ],
            "inputs": {
                "input": "{SfMColorizing_2.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ConvertSfMFormat_1": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                4140,
                -31
            ],
            "inputs": {
                "input": "{ExportImages_2.outputSfMData}",
                "fileExt": "json",
                "describerTypes": "{TracksBuilding_2.describerTypes}",
                "structure": false,
                "observations": false
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ConvertSfMFormat_2": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                705,
                -573
            ],
            "inputs": {
                "input": "{ExportImages_3.outputSfMData}",
                "fileExt": "json",
                "structure": false,
                "observations": false
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ConvertSfMFormat_3": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                2499,
                -553
            ],
            "inputs": {
                "input": "{ExportImages_4.outputSfMData}",
                "fileExt": "json",
                "structure": false,
                "observations": false
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "CopyFiles_1": {
            "nodeType": "CopyFiles",
            "position": [
                4251,
                299
            ],
            "inputs": {
                "inputFiles": [
                    "{ExportDistortion_1.distortionNukeNode}",
                    "{ExportAlembic_1.output}"
                ]
            }
        },
        "ExportAlembic_1": {
            "nodeType": "ExportAlembic",
            "position": [
                3957,
                188
            ],
            "inputs": {
                "input": "{ExportImages_2.outputSfMData}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportDistortion_1": {
            "nodeType": "ExportDistortion",
            "position": [
                3957,
                384
            ],
            "inputs": {
                "input": "{ConvertDistortion_1.output}",
                "exportLensGridsUndistorted": false
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportImages_2": {
            "nodeType": "ExportImages",
            "position": [
                3747,
                188
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_2.input}",
                "target": "{IntrinsicsTransforming_2.output}",
                "namingMode": "keep"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportImages_3": {
            "nodeType": "ExportImages",
            "position": [
                495,
                -573
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_3.input}",
                "target": "{IntrinsicsTransforming_3.output}",
                "namingMode": "keep"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportImages_4": {
            "nodeType": "ExportImages",
            "position": [
                2289,
                -553
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_4.input}",
                "target": "{IntrinsicsTransforming_4.output}",
                "namingMode": "keep"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "FeatureExtraction_1": {
            "nodeType": "FeatureExtraction",
            "position": [
                350,
                -48
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "masksFolder": "{MaskEroding_1.output}",
                "maskExtension": "exr"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "FeatureMatching_1": {
            "nodeType": "FeatureMatching",
            "position": [
                770,
                -48
            ],
            "inputs": {
                "input": "{ImageMatching_1.input}",
                "featuresFolders": "{ImageMatching_1.featuresFolders}",
                "imagePairsList": "{ImageMatching_1.output}",
                "describerTypes": "{FeatureExtraction_1.describerTypes}"
            },
            "internalInputs": {
                "label": "FeatureMatchingKeyframes",
                "color": "#575963"
            }
        },
        "FeatureMatching_2": {
            "nodeType": "FeatureMatching",
            "position": [
                2270,
                354
            ],
            "inputs": {
                "input": "{ImageMatching_2.input}",
                "featuresFolders": "{ImageMatching_2.featuresFolders}",
                "imagePairsList": "{ImageMatching_2.output}"
            },
            "internalInputs": {
                "label": "FeatureMatchingAllFrames",
                "color": "#80766f"
            }
        },
        "FeatureMatching_3": {
            "nodeType": "FeatureMatching",
            "position": [
                2277,
                188
            ],
            "inputs": {
                "input": "{ImageMatchingMultiSfM_1.outputCombinedSfM}",
                "featuresFolders": "{ImageMatchingMultiSfM_1.featuresFolders}",
                "imagePairsList": "{ImageMatchingMultiSfM_1.output}",
                "describerTypes": "{FeatureExtraction_1.describerTypes}"
            },
            "internalInputs": {
                "label": "FeatureMatchingFramesToKeyframes",
                "color": "#80766f"
            }
        },
        "ImageMatchingMultiSfM_1": {
            "nodeType": "ImageMatchingMultiSfM",
            "position": [
                2067,
                188
            ],
            "inputs": {
                "input": "{SfMSurveyInjecting_1.output}",
                "inputB": "{SfMExpanding_4.output}",
                "featuresFolders": [
                    "{FeatureExtraction_1.output}"
                ],
                "method": "VocabularyTree",
                "matchingMode": "a/b",
                "nbMatches": 20
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ImageMatching_1": {
            "nodeType": "ImageMatching",
            "position": [
                560,
                -48
            ],
            "inputs": {
                "input": "{KeyframeSelection_1.outputSfMDataKeyframes}",
                "featuresFolders": [
                    "{FeatureExtraction_1.output}"
                ],
                "method": "Exhaustive"
            },
            "internalInputs": {
                "label": "ImageMatchingKeyframes",
                "color": "#575963"
            }
        },
        "ImageMatching_2": {
            "nodeType": "ImageMatching",
            "position": [
                2060,
                354
            ],
            "inputs": {
                "input": "{SfMSurveyInjecting_1.output}",
                "featuresFolders": [
                    "{FeatureExtraction_1.output}"
                ],
                "method": "Sequential",
                "nbNeighbors": 20
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ImageSegmentationSam3_1": {
            "nodeType": "ImageSegmentationSam3",
            "position": [
                -1115,
                282
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "prompt": "person",
                "maskInvert": true
            }
        },
        "IntrinsicsTransforming_2": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                3537,
                188
            ],
            "inputs": {
                "input": "{SfMColorizing_2.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "IntrinsicsTransforming_3": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                285,
                -573
            ],
            "inputs": {
                "input": "{SfMSplitReconstructed_1.reconstructedOutput}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "IntrinsicsTransforming_4": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                2079,
                -553
            ],
            "inputs": {
                "input": "{SfMSplitReconstructed_2.reconstructedOutput}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "KeyframeSelection_1": {
            "nodeType": "KeyframeSelection",
            "position": [
                140,
                -48
            ],
            "inputs": {
                "inputPaths": [
                    "{SfMSurveyInjecting_1.output}"
                ],
                "maskPaths": [
                    "{MaskEroding_1.output}"
                ],
                "selectionMethod": {
                    "useSmartSelection": true,
                    "regularSelection": {
                        "minFrameStep": 12,
                        "maxFrameStep": 0,
                        "maxNbOutFrames": 0
                    },
                    "smartSelection": {
                        "pxDisplacement": 10.0,
                        "minNbOutFrames": 70,
                        "maxNbOutFrames": 2000,
                        "rescaledWidthSharpness": 720,
                        "rescaledWidthFlow": 720,
                        "sharpnessWindowSize": 200,
                        "flowCellSize": 90,
                        "minBlockSize": 10
                    }
                }
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "Lidarmesh_1": {
            "nodeType": "InputFile",
            "position": [
                -1207,
                -77
            ],
            "inputs": {},
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "MaskEroding_1": {
            "nodeType": "MaskEroding",
            "position": [
                -719,
                329
            ],
            "inputs": {
                "input": "{MaskProcessing_1.output}",
                "radius": 15
            }
        },
        "MaskProcessing_1": {
            "nodeType": "MaskProcessing",
            "position": [
                -906,
                336
            ],
            "inputs": {
                "inputs": [
                    "{VideoSegmentationSam3_1.output}",
                    "{ImageSegmentationSam3_1.output}"
                ]
            }
        },
        "PrevisualisationLidar_1": {
            "nodeType": "Backdrop",
            "position": [
                21,
                -684
            ],
            "internalInputs": {
                "label": "Previz Lidar",
                "nodeWidth": 1122,
                "nodeHeight": 282
            }
        },
        "ScenePreview_1": {
            "nodeType": "ScenePreview",
            "position": [
                4350,
                -31
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_1.output}",
                "model": "{Lidarmesh_1.inputFile}",
                "undistortedImages": "{ExportImages_2.output}",
                "masks": "{MaskEroding_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ScenePreview_2": {
            "nodeType": "ScenePreview",
            "position": [
                915,
                -573
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_2.output}",
                "model": "{Lidarmesh_1.inputFile}",
                "undistortedImages": "{ExportImages_3.output}",
                "masks": "{MaskEroding_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ScenePreview_3": {
            "nodeType": "ScenePreview",
            "position": [
                2709,
                -553
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_3.output}",
                "model": "{Lidarmesh_1.inputFile}",
                "undistortedImages": "{ExportImages_4.output}",
                "masks": "{MaskEroding_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMBootStrapping_1": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1190,
                -48
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "method": "mesh_single",
                "tracksFilename": "{TracksBuilding_1.output}",
                "meshFilename": "{Lidarmesh_1.inputFile}",
                "pairs": "empty"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "SfMColorizing_2": {
            "nodeType": "SfMColorizing",
            "position": [
                3327,
                188
            ],
            "inputs": {
                "input": "{SfMExpanding_3.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMExpanding_2": {
            "nodeType": "SfMExpanding",
            "position": [
                2907,
                188
            ],
            "inputs": {
                "input": "{TracksBuilding_2.input}",
                "tracksFilename": "{TracksMerging_1.output}",
                "meshFilename": "{Lidarmesh_1.inputFile}",
                "localizerEstimatorError": 4.0,
                "nbFirstUnstableCameras": 0,
                "maxImagesPerGroup": 0,
                "bundleAdjustmentMaxOutliers": 5000000,
                "minNumberOfObservationsForTriangulation": 3,
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the complete camera tracking sequence.",
                "label": "SfmExpanding",
                "color": "#80766f"
            }
        },
        "SfMExpanding_3": {
            "nodeType": "SfMExpanding",
            "position": [
                3117,
                188
            ],
            "inputs": {
                "input": "{SfMExpanding_2.output}",
                "tracksFilename": "{SfMExpanding_2.tracksFilename}",
                "meshFilename": "{SfMExpanding_2.meshFilename}",
                "localizerEstimatorError": 4.0,
                "useLocalBA": false,
                "useTemporalConstraint": true,
                "nbFirstUnstableCameras": 0,
                "maxImagesPerGroup": 0,
                "bundleAdjustmentMaxOutliers": 5000000,
                "minNumberOfObservationsForTriangulation": 3,
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the complete camera tracking sequence.",
                "label": "Filtering",
                "color": "#80766f"
            }
        },
        "SfMExpanding_4": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                -48
            ],
            "inputs": {
                "input": "{SfMBootStrapping_1.output}",
                "tracksFilename": "{SfMBootStrapping_1.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_1.meshFilename}",
                "localizerEstimatorError": 4.0,
                "maxImagesPerGroup": 1,
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the keyframes.",
                "label": "SfMExpandingKeys",
                "color": "#575963"
            }
        },
        "SfMFilter_1": {
            "nodeType": "SfMFilter",
            "position": [
                -788,
                10
            ],
            "inputs": {
                "inputFile": "{SfMPoseInjecting_1.output}",
                "expression": "frameId == 977"
            },
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "SfMPoseInjecting_1": {
            "nodeType": "SfMPoseInjecting",
            "position": [
                -998,
                10
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "offset": 977
            },
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "SfMSplitReconstructed_1": {
            "nodeType": "SfMSplitReconstructed",
            "position": [
                75,
                -573
            ],
            "inputs": {
                "input": "{TransferInjectedPose_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMSplitReconstructed_2": {
            "nodeType": "SfMSplitReconstructed",
            "position": [
                1869,
                -553
            ],
            "inputs": {
                "input": "{SfMExpanding_4.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMSurveyInjecting_1": {
            "nodeType": "SfMSurveyInjecting",
            "position": [
                -368,
                10
            ],
            "inputs": {
                "input": "{TransferInjectedPose_1.output}"
            },
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "TracksBuilding_1": {
            "nodeType": "TracksBuilding",
            "position": [
                980,
                -48
            ],
            "inputs": {
                "input": "{FeatureMatching_1.input}",
                "featuresFolders": "{FeatureMatching_1.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_1.output}"
                ],
                "describerTypes": "{FeatureMatching_1.describerTypes}",
                "filterTrackForks": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "TracksBuilding_2": {
            "nodeType": "TracksBuilding",
            "position": [
                2487,
                188
            ],
            "inputs": {
                "input": "{FeatureMatching_3.input}",
                "featuresFolders": "{FeatureMatching_3.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_2.output}",
                    "{FeatureMatching_3.output}"
                ],
                "describerTypes": "{FeatureMatching_3.describerTypes}",
                "minInputTrackLength": 5,
                "filterTrackForks": true
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "TracksMerging_1": {
            "nodeType": "TracksMerging",
            "position": [
                2697,
                188
            ],
            "inputs": {
                "inputs": [
                    "{TracksBuilding_2.output}"
                ]
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "TransferInjectedPose_1": {
            "nodeType": "SfMTransfer",
            "position": [
                -578,
                10
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "reference": "{SfMFilter_1.outputSfMData_selected}"
            },
            "internalInputs": {
                "color": "#c49156"
            }
        },
        "VideoSegmentationSam3_1": {
            "nodeType": "VideoSegmentationSam3",
            "position": [
                -1114,
                402
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "prompt": "vehicle",
                "combineFwdAndBwdSeg": true,
                "maskInvert": true
            }
        }
    }
}