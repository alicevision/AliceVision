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
            "SfMTriangulation": "1.0",
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
                "label": "CameraInitLensGrid",
                "color": "#302e2e"
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
                4679,
                219
            ],
            "inputs": {
                "input": "{SfMColorizing_2.output}",
                "fileExt": "json",
                "describerTypes": "{TracksBuilding_2.describerTypes}",
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
                5079,
                119
            ],
            "inputs": {
                "inputFiles": [
                    "{Texturing_1.output}",
                    "{ScenePreview_1.output}",
                    "{ExportDistortion_1.output}",
                    "{ExportAlembic_1.output}",
                    "{ExportImages_2.output}"
                ]
            }
        },
        "DepthMapFilter_1": {
            "nodeType": "DepthMapFilter",
            "position": [
                4079,
                19
            ],
            "inputs": {
                "input": "{DepthMap_1.input}",
                "depthMapsFolder": "{DepthMap_1.output}"
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "DepthMap_1": {
            "nodeType": "DepthMap",
            "position": [
                3879,
                19
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.output}",
                "imagesFolder": "{ExportImages_1.output}",
                "downscale": 1
            },
            "internalInputs": {
                "color": "#3f3138"
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
                3456,
                206
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
                3675,
                17
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.input}",
                "target": "{IntrinsicsTransforming_1.output}",
                "masksFolders": [
                    "{ImageSegmentationSam3_1.output}"
                ],
                "maskExtension": "exr"
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "ExportImages_2": {
            "nodeType": "ExportImages",
            "position": [
                3250,
                194
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
                1826,
                409
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
                1826,
                209
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
                1626,
                209
            ],
            "inputs": {
                "input": "{ApplyCalibration_1.output}",
                "inputB": "{SfMExpanding_1.output}",
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
                1626,
                409
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
        "ImageSegmentationSam3_1": {
            "nodeType": "ImageSegmentationSam3",
            "position": [
                197.0,
                201.0
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
                3474,
                17
            ],
            "inputs": {
                "input": "{SfMTriangulation_1.output}"
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "IntrinsicsTransforming_2": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                3050,
                194
            ],
            "inputs": {
                "input": "{SfMColorizing_2.output}"
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
                4679,
                19
            ],
            "inputs": {
                "input": "{MeshFiltering_1.outputMesh}",
                "simplificationFactor": 0.05
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "MeshFiltering_1": {
            "nodeType": "MeshFiltering",
            "position": [
                4479,
                19
            ],
            "inputs": {
                "inputMesh": "{Meshing_1.outputMesh}",
                "filterLargeTrianglesFactor": 10.0
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "Meshing_1": {
            "nodeType": "Meshing",
            "position": [
                4279,
                19
            ],
            "inputs": {
                "input": "{DepthMapFilter_1.input}",
                "depthMapsFolder": "{DepthMapFilter_1.output}",
                "estimateSpaceFromSfM": false,
                "minStep": 1,
                "fullWeight": 10.0,
                "saveRawDensePointCloud": true
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "RelativePoseEstimating_1": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                0
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "countIterations": 50000,
                "minInliers": 100,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ScenePreview_1": {
            "nodeType": "ScenePreview",
            "position": [
                4879,
                219
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
                1200,
                0
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_1.input}",
                "tracksFilename": "{RelativePoseEstimating_1.tracksFilename}",
                "pairs": "{RelativePoseEstimating_1.output}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "SfMColorizing_2": {
            "nodeType": "SfMColorizing",
            "position": [
                2812.5,
                202.0
            ],
            "inputs": {
                "input": "{SfMTransform_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMExpanding_1": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                0
            ],
            "inputs": {
                "input": "{SfMBootStrapping_1.output}",
                "tracksFilename": "{SfMBootStrapping_1.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_1.meshFilename}",
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the keyframes.",
                "label": "SfMExpandingKeys",
                "color": "#575963"
            }
        },
        "SfMExpanding_2": {
            "nodeType": "SfMExpanding",
            "position": [
                2426,
                209
            ],
            "inputs": {
                "input": "{TracksBuilding_2.input}",
                "tracksFilename": "{TracksMerging_1.output}",
                "meshFilename": "{SfMExpanding_1.meshFilename}",
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
                3077,
                22
            ],
            "inputs": {
                "input": "{KeyframeSelection_1.outputSfMDataKeyframes}",
                "reference": "{SfMColorizing_2.output}",
                "transferLandmarks": false
            },
            "internalInputs": {
                "comment": "Transfer pose from final camera tracking into the keyframes-only scene.",
                "color": "#3f3138"
            }
        },
        "SfMTransform_1": {
            "nodeType": "SfMTransform",
            "position": [
                2623.5,
                204.0
            ],
            "inputs": {
                "input": "{SfMExpanding_2.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "SfMTriangulation_1": {
            "nodeType": "SfMTriangulation",
            "position": [
                3277,
                22
            ],
            "inputs": {
                "input": "{SfMTransfer_1.output}",
                "featuresFolders": "{TracksBuilding_1.featuresFolders}",
                "matchesFolders": "{TracksBuilding_1.matchesFolders}",
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "Texturing_1": {
            "nodeType": "Texturing",
            "position": [
                4879,
                19
            ],
            "inputs": {
                "input": "{Meshing_1.output}",
                "imagesFolder": "{ExportImages_1.output}",
                "inputMesh": "{MeshDecimate_1.output}"
            },
            "internalInputs": {
                "color": "#3f3138"
            }
        },
        "TracksBuilding_1": {
            "nodeType": "TracksBuilding",
            "position": [
                800,
                0
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
                2026,
                209
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
                2226,
                209
            ],
            "inputs": {
                "inputs": [
                    "{TracksBuilding_2.output}",
                    "{SfMExpanding_1.tracksFilename}"
                ]
            },
            "internalInputs": {
                "color": "#80766f"
            }
        }
    }
}
