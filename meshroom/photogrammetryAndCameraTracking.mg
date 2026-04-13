{
    "header": {
        "releaseVersion": "2026.1.0+develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "ApplyCalibration": "1.0",
            "CameraInit": "12.1",
            "CheckerboardDetection": "2.0",
            "ConvertSfMFormat": "2.0",
            "CopyFiles": "1.3",
            "Depthmap": "5.1",
            "DepthMapFilter": "4.0",
            "DistortionCalibration": "6.1",
            "ExportAlembic": "1.0",
            "ExportDistortion": "2.0",
            "ExportImages": "1.1",
            "FeatureExtraction": "1.3",
            "FeatureMatching": "2.0",
            "ImageMatching": "2.0",
            "ImageMatchingMultiSfM": "1.0",
            "ImageSegmentationSam3": "1.0",
            "IntrinsicsTransforming": "1.1",
            "KeyframeSelection": "5.0",
            "MeshDecimate": "1.0",
            "MeshFiltering": "3.0",
            "Meshing": "7.0",
            "RelativePoseEstimating": "3.1",
            "ScenePreview": "2.0",
            "SfMBootStrapping": "4.2",
            "SfMColorizing": "1.0",
            "SfMExpanding": "2.3",
            "SfMTransfer": "2.1",
            "SfMTransform": "3.2",
            "Texturing": "6.0",
            "TracksBuilding": "1.0",
            "TracksMerging": "3.0"
        },
        "template": true
    },
    "graph": {
        "ApplyCalibration_1": {
            "nodeType": "ApplyCalibration",
            "position": [
                0,
                0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "calibration": "{DistortionCalibration_1.output}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -200,
                0
            ],
            "inputs": {
                "isSequence": true
            },
            "internalInputs": {
                "label": "InitShot",
                "color": "#575963"
            }
        },
        "CameraInit_2": {
            "nodeType": "CameraInit",
            "position": [
                -600,
                -160
            ],
            "inputs": {},
            "internalInputs": {
                "label": "InitLensGrid",
                "color": "#302e2e"
            }
        },
        "CameraInit_3": {
            "nodeType": "CameraInit",
            "position": [
                -600,
                -500
            ],
            "inputs": {},
            "internalInputs": {
                "label": "InitPhotogrammetry",
                "color": "#384a55"
            }
        },
        "CheckerboardDetection_1": {
            "nodeType": "CheckerboardDetection",
            "position": [
                -400,
                -160
            ],
            "inputs": {
                "input": "{CameraInit_2.output}",
                "useNestedGrids": true,
                "exportDebugImages": true
            },
            "internalInputs": {
                "color": "#302e2e"
            }
        },
        "ConvertSfMFormat_1": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                4802,
                171
            ],
            "inputs": {
                "input": "{SfMTransfer_1.output}",
                "fileExt": "sfm",
                "describerTypes": "{TracksBuilding_3.describerTypes}",
                "structure": false,
                "observations": false
            },
            "internalInputs": {
                "color": "#4c594c"
            }
        },
        "CopyFiles_1": {
            "nodeType": "CopyFiles",
            "position": [
                5368,
                -145
            ],
            "inputs": {
                "inputFiles": [
                    "{ScenePreview_1.output}",
                    "{ExportDistortion_1.output}",
                    "{Texturing_2.output}",
                    "{ExportImages_2.output}",
                    "{ExportAlembic_1.output}"
                ]
            }
        },
        "DepthMapFilter_2": {
            "nodeType": "DepthMapFilter",
            "position": [
                3065,
                -529
            ],
            "inputs": {
                "input": "{DepthMap_2.input}",
                "depthMapsFolder": "{DepthMap_2.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "DepthMap_2": {
            "nodeType": "DepthMap",
            "position": [
                2865,
                -529
            ],
            "inputs": {
                "input": "{ExportImages_1.input}",
                "imagesFolder": "{ExportImages_1.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "DistortionCalibration_1": {
            "nodeType": "DistortionCalibration",
            "position": [
                -200,
                -160
            ],
            "inputs": {
                "input": "{CheckerboardDetection_1.input}",
                "checkerboards": "{CheckerboardDetection_1.output}"
            },
            "internalInputs": {
                "color": "#302e2e"
            }
        },
        "ExportAlembic_1": {
            "nodeType": "ExportAlembic",
            "position": [
                4522.0,
                218.0
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
                0,
                -160
            ],
            "inputs": {
                "input": "{DistortionCalibration_1.output}"
            },
            "internalInputs": {
                "color": "#302e2e"
            }
        },
        "ExportImages_1": {
            "nodeType": "ExportImages",
            "position": [
                2661.5,
                -526.0
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.input}",
                "target": "{IntrinsicsTransforming_1.output}",
                "maskExtension": "exr"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "ExportImages_2": {
            "nodeType": "ExportImages",
            "position": [
                4316.0,
                206.0
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
        "FeatureExtraction_1": {
            "nodeType": "FeatureExtraction",
            "position": [
                400,
                200
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "masksFolder": "{ImageSegmentationSam3_1.output}",
                "maskExtension": "exr"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "FeatureExtraction_2": {
            "nodeType": "FeatureExtraction",
            "position": [
                -400,
                -500
            ],
            "inputs": {
                "input": "{CameraInit_3.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "FeatureMatching_1": {
            "nodeType": "FeatureMatching",
            "position": [
                600,
                0
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
                2876,
                312
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
                2876,
                157
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
        "FeatureMatching_4": {
            "nodeType": "FeatureMatching",
            "position": [
                0,
                -500
            ],
            "inputs": {
                "input": "{ImageMatching_3.input}",
                "featuresFolders": "{ImageMatching_3.featuresFolders}",
                "imagePairsList": "{ImageMatching_3.output}",
                "describerTypes": "{FeatureExtraction_2.describerTypes}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "FeatureMatching_5": {
            "nodeType": "FeatureMatching",
            "position": [
                1627,
                -246
            ],
            "inputs": {
                "input": "{ImageMatchingMultiSfM_2.outputCombinedSfM}",
                "featuresFolders": "{ImageMatchingMultiSfM_2.featuresFolders}",
                "imagePairsList": "{ImageMatchingMultiSfM_2.output}",
                "describerTypes": "{FeatureExtraction_1.describerTypes}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ImageMatchingMultiSfM_1": {
            "nodeType": "ImageMatchingMultiSfM",
            "position": [
                2673,
                139
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "inputB": "{SfMExpanding_2.output}",
                "featuresFolders": "{TracksBuilding_2.featuresFolders}",
                "method": "VocabularyTree",
                "matchingMode": "a/b",
                "nbMatches": 20
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ImageMatchingMultiSfM_2": {
            "nodeType": "ImageMatchingMultiSfM",
            "position": [
                1431,
                -238
            ],
            "inputs": {
                "input": "{KeyframeSelection_1.outputSfMDataKeyframes}",
                "inputB": "{SfMColorizing_2.output}",
                "featuresFolders": [
                    "{FeatureExtraction_2.output}",
                    "{FeatureExtraction_1.output}"
                ],
                "method": "Exhaustive",
                "matchingMode": "a/b"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ImageMatching_1": {
            "nodeType": "ImageMatching",
            "position": [
                400,
                0
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
                2676,
                312
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
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
        "ImageMatching_3": {
            "nodeType": "ImageMatching",
            "position": [
                -200,
                -500
            ],
            "inputs": {
                "input": "{FeatureExtraction_2.input}",
                "featuresFolders": [
                    "{FeatureExtraction_2.output}"
                ]
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "ImageSegmentationSam3_1": {
            "nodeType": "ImageSegmentationSam3",
            "position": [
                200.0,
                200.0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "maskInvert": true,
                "keepFilename": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "IntrinsicsTransforming_1": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                2460.5,
                -526.0
            ],
            "inputs": {
                "input": "{SfMColorizing_2.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "IntrinsicsTransforming_2": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                4116.0,
                206.0
            ],
            "inputs": {
                "input": "{SfMTransfer_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "KeyframeSelection_1": {
            "nodeType": "KeyframeSelection",
            "position": [
                200,
                0
            ],
            "inputs": {
                "inputPaths": [
                    "{ApplyCalibration_1.output}"
                ]
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "MeshDecimate_1": {
            "nodeType": "MeshDecimate",
            "position": [
                4164,
                24
            ],
            "inputs": {
                "input": "{MeshFiltering_2.outputMesh}",
                "simplificationFactor": 0.05
            },
            "internalInputs": {
                "color": "#4c594c"
            }
        },
        "MeshFiltering_2": {
            "nodeType": "MeshFiltering",
            "position": [
                3465,
                -529
            ],
            "inputs": {
                "inputMesh": "{Meshing_2.outputMesh}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "Meshing_2": {
            "nodeType": "Meshing",
            "position": [
                3265,
                -529
            ],
            "inputs": {
                "input": "{DepthMapFilter_2.input}",
                "depthMapsFolder": "{DepthMapFilter_2.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "RelativePoseEstimating_1": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                400,
                -500
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "imagePairsList": "{FeatureMatching_4.imagePairsList}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "ScenePreview_1": {
            "nodeType": "ScenePreview",
            "position": [
                4991,
                150
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_1.output}",
                "model": "{MeshDecimate_1.output}",
                "undistortedImages": "{ExportImages_2.output}",
                "masks": "{ImageSegmentationSam3_1.output}"
            },
            "internalInputs": {
                "color": "#4c594c"
            }
        },
        "SfMBootStrapping_1": {
            "nodeType": "SfMBootStrapping",
            "position": [
                600,
                -500
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_1.input}",
                "tracksFilename": "{RelativePoseEstimating_1.tracksFilename}",
                "pairs": "{RelativePoseEstimating_1.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "SfMColorizing_1": {
            "nodeType": "SfMColorizing",
            "position": [
                3663,
                132
            ],
            "inputs": {
                "input": "{SfMExpanding_3.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMColorizing_2": {
            "nodeType": "SfMColorizing",
            "position": [
                1183,
                -500
            ],
            "inputs": {
                "input": "{SfMTransform_1.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "SfMExpanding_1": {
            "nodeType": "SfMExpanding",
            "position": [
                800,
                -500
            ],
            "inputs": {
                "input": "{SfMBootStrapping_1.output}",
                "tracksFilename": "{SfMBootStrapping_1.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_1.meshFilename}"
            },
            "internalInputs": {
                "label": "SfMExpandingPhotog",
                "color": "#384a55"
            }
        },
        "SfMExpanding_2": {
            "nodeType": "SfMExpanding",
            "position": [
                2295,
                -105
            ],
            "inputs": {
                "input": "{TracksBuilding_2.input}",
                "tracksFilename": "{TracksMerging_2.output}",
                "lockScenePreviouslyReconstructed": true,
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the keyframes.",
                "label": "SfMExpandingKeys",
                "color": "#575963"
            }
        },
        "SfMExpanding_3": {
            "nodeType": "SfMExpanding",
            "position": [
                3468,
                133
            ],
            "inputs": {
                "input": "{TracksBuilding_3.input}",
                "tracksFilename": "{TracksMerging_1.output}",
                "nbFirstUnstableCameras": 0,
                "maxImagesPerGroup": 0,
                "bundleAdjustmentMaxOutliers": 5000000,
                "minNumberOfObservationsForTriangulation": 3,
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the complete camera tracking sequence.",
                "label": "SfMExpandingAll",
                "color": "#80766f"
            }
        },
        "SfMTransfer_1": {
            "nodeType": "SfMTransfer",
            "position": [
                3903.0,
                207.0
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "reference": "{SfMColorizing_1.output}"
            },
            "internalInputs": {
                "label": "ShotWithPoses",
                "color": "#80766f"
            }
        },
        "SfMTransform_1": {
            "nodeType": "SfMTransform",
            "position": [
                997,
                -501
            ],
            "inputs": {
                "input": "{SfMExpanding_1.output}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "Texturing_2": {
            "nodeType": "Texturing",
            "position": [
                3665,
                -529
            ],
            "inputs": {
                "input": "{Meshing_2.output}",
                "imagesFolder": "{DepthMap_2.imagesFolder}",
                "inputMesh": "{MeshFiltering_2.outputMesh}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "TracksBuilding_1": {
            "nodeType": "TracksBuilding",
            "position": [
                200,
                -500
            ],
            "inputs": {
                "input": "{FeatureMatching_4.input}",
                "featuresFolders": "{FeatureMatching_4.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_4.output}"
                ],
                "describerTypes": "{FeatureMatching_4.describerTypes}"
            },
            "internalInputs": {
                "color": "#384a55"
            }
        },
        "TracksBuilding_2": {
            "nodeType": "TracksBuilding",
            "position": [
                1888,
                -67
            ],
            "inputs": {
                "input": "{FeatureMatching_5.input}",
                "featuresFolders": "{FeatureMatching_5.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_1.output}",
                    "{FeatureMatching_5.output}"
                ],
                "describerTypes": "{FeatureMatching_5.describerTypes}",
                "filterTrackForks": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "TracksBuilding_3": {
            "nodeType": "TracksBuilding",
            "position": [
                3076,
                157
            ],
            "inputs": {
                "input": "{FeatureMatching_3.input}",
                "featuresFolders": "{FeatureMatching_3.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_3.output}",
                    "{FeatureMatching_2.output}"
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
                3273,
                102
            ],
            "inputs": {
                "inputs": [
                    "{TracksBuilding_3.output}",
                    "{SfMExpanding_2.tracksFilename}"
                ]
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "TracksMerging_2": {
            "nodeType": "TracksMerging",
            "position": [
                2099,
                -84
            ],
            "inputs": {
                "inputs": [
                    "{TracksBuilding_2.output}",
                    "{TracksBuilding_1.output}"
                ]
            },
            "internalInputs": {
                "color": "#575963"
            }
        }
    }
}
