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
            "DistortionCalibration": "6.1",
            "ExportAlembic": "1.0",
            "ExportDistortion": "2.0",
            "ExportImages": "1.1",
            "GeometricFilterEstimating": "1.0",
            "ImageSegmentationSam3": "1.0",
            "IntrinsicsTransforming": "1.1",
            "KeyframeSelection": "5.0",
            "MatchMasking": "1.0",
            "RelativePoseEstimating": "3.1",
            "RomaMatcher": "1.0",
            "RomaReducer": "1.0",
            "RomaSampler": "1.0",
            "ScenePreview": "2.0",
            "SfMBootStrapping": "4.2",
            "SfMColorizing": "1.0",
            "SfMExpanding": "2.3",
            "SfMTransform": "3.2",
            "StarListing": "1.0",
            "TracksBuilding": "1.0"
        },
        "template": true
    },
    "graph": {
        "ApplyCalibration_1": {
            "nodeType": "ApplyCalibration",
            "position": [
                -6,
                34
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
                -216,
                34
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
                3564,
                34
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.input}",
                "fileExt": "json",
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
                4033,
                -1
            ],
            "inputs": {
                "inputFiles": [
                    "{ScenePreview_1.output}",
                    "{ExportDistortion_1.output}",
                    "{ExportAlembic_1.output}",
                    "{ExportImages_1.output}"
                ]
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
                3791,
                -78
            ],
            "inputs": {
                "input": "{ExportImages_1.outputSfMData}"
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
                3569,
                -102
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.input}",
                "target": "{IntrinsicsTransforming_1.output}",
                "namingMode": "keep"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "GeometricFilterEstimating_1": {
            "nodeType": "GeometricFilterEstimating",
            "position": [
                1464,
                34
            ],
            "inputs": {
                "input": "{RomaReducer_1.inputSfMData}",
                "featuresFolders": [
                    "{RomaReducer_1.featuresFolder}"
                ],
                "matchesFolders": [
                    "{RomaReducer_1.matchesFolder}"
                ],
                "describerTypes": "{RomaReducer_1.describerTypes}",
                "geometricError": 16.0
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ImageSegmentationSam3_1": {
            "nodeType": "ImageSegmentationSam3",
            "position": [
                205.0,
                210.0
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
                3354,
                34
            ],
            "inputs": {
                "input": "{SfMColorizing_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "KeyframeSelection_1": {
            "nodeType": "KeyframeSelection",
            "position": [
                204,
                34
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
        "MatchMasking_1": {
            "nodeType": "MatchMasking",
            "position": [
                834,
                34
            ],
            "inputs": {
                "inputSfMData": "{RomaMatcher_1.inputSfMData}",
                "imagePairsList": "{RomaMatcher_1.imagePairsList}",
                "warpFolder": "{RomaMatcher_1.outputWarpFolder}",
                "certaintyFolder": "{RomaMatcher_1.outputCertaintyFolder}",
                "masksFolder": "{ImageSegmentationSam3_1.output}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RelativePoseEstimating_1": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                2304,
                34
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "countIterations": 50000,
                "minInliers": 100,
                "distanceThreshold": 0.0,
                "imagePairsList": "{RomaReducer_2.imagePairsList}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RomaMatcher_1": {
            "nodeType": "RomaMatcher",
            "position": [
                624,
                34
            ],
            "inputs": {
                "inputSfMData": "{StarListing_1.inputSfMData}",
                "imagePairsList": "{StarListing_1.imagePairsList}",
                "checkLoops": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RomaReducer_1": {
            "nodeType": "RomaReducer",
            "position": [
                1254,
                34
            ],
            "inputs": {
                "inputSfMData": "{RomaSampler_1.inputSfMData}",
                "imagePairsList": "{RomaSampler_1.imagePairsList}",
                "samplesFolder": "{RomaSampler_1.samplesFolder}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RomaReducer_2": {
            "nodeType": "RomaReducer",
            "position": [
                1884,
                34
            ],
            "inputs": {
                "inputSfMData": "{RomaSampler_2.inputSfMData}",
                "imagePairsList": "{RomaSampler_2.imagePairsList}",
                "samplesFolder": "{RomaSampler_2.samplesFolder}",
                "describerTypes": "{RomaSampler_2.describerTypes}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RomaSampler_1": {
            "nodeType": "RomaSampler",
            "position": [
                1044,
                34
            ],
            "inputs": {
                "inputSfMData": "{MatchMasking_1.inputSfMData}",
                "imagePairsList": "{MatchMasking_1.imagePairsList}",
                "warpFolder": "{MatchMasking_1.warpFolder}",
                "certaintyFolder": "{MatchMasking_1.outputCertaintyFolder}",
                "maxMatches": 5000
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "RomaSampler_2": {
            "nodeType": "RomaSampler",
            "position": [
                1674,
                34
            ],
            "inputs": {
                "inputSfMData": "{GeometricFilterEstimating_1.input}",
                "imagePairsList": "{RomaReducer_1.imagePairsList}",
                "warpFolder": "{RomaSampler_1.warpFolder}",
                "certaintyFolder": "{RomaSampler_1.certaintyFolder}",
                "maxMatches": 2500,
                "minCertainty": 0.2,
                "filtersFolder": "{GeometricFilterEstimating_1.output}",
                "describerTypes": "{GeometricFilterEstimating_1.describerTypes}"
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "ScenePreview_1": {
            "nodeType": "ScenePreview",
            "position": [
                3793,
                10
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_1.output}",
                "model": "{ConvertSfMFormat_1.input}",
                "undistortedImages": "{ExportImages_1.output}",
                "masks": "{ImageSegmentationSam3_1.output}"
            },
            "internalInputs": {
                "color": "#4c594c"
            }
        },
        "SfMBootStrapping_1": {
            "nodeType": "SfMBootStrapping",
            "position": [
                2514,
                34
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
        "SfMColorizing_1": {
            "nodeType": "SfMColorizing",
            "position": [
                3144,
                34
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
                2724,
                34
            ],
            "inputs": {
                "input": "{SfMBootStrapping_1.output}",
                "tracksFilename": "{SfMBootStrapping_1.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_1.meshFilename}",
                "minAngleForTriangulation": 1.0,
                "minAngleForLandmark": 0.5,
                "maxReprojectionError": 16.0
            },
            "internalInputs": {
                "comment": "Estimate cameras parameters for the keyframes.",
                "label": "SfMExpandingKeys",
                "color": "#575963"
            }
        },
        "SfMTransform_1": {
            "nodeType": "SfMTransform",
            "position": [
                2934,
                34
            ],
            "inputs": {
                "input": "{SfMExpanding_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "StarListing_1": {
            "nodeType": "StarListing",
            "position": [
                414,
                34
            ],
            "inputs": {
                "inputSfMData": "{KeyframeSelection_1.inputPaths[0]}",
                "keySfMData": "{KeyframeSelection_1.outputSfMDataKeyframes}",
                "radiusKeyFrames": 5
            },
            "internalInputs": {
                "color": "#575963"
            }
        },
        "TracksBuilding_1": {
            "nodeType": "TracksBuilding",
            "position": [
                2094,
                34
            ],
            "inputs": {
                "input": "{RomaReducer_2.inputSfMData}",
                "featuresFolders": [
                    "{RomaReducer_2.featuresFolder}"
                ],
                "matchesFolders": [
                    "{RomaReducer_2.matchesFolder}"
                ],
                "describerTypes": "{RomaReducer_2.describerTypes}",
                "filterTrackForks": true
            },
            "internalInputs": {
                "color": "#575963"
            }
        }
    }
}
