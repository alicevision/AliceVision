{
    "header": {
        "releaseVersion": "2026.1.0+develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "ConvertDistortion": "1.0",
            "ConvertSfMFormat": "2.0",
            "CopyFiles": "1.3",
            "ExportAlembic": "1.0",
            "ExportDistortion": "2.0",
            "ExportImages": "1.1",
            "FeatureExtraction": "1.3",
            "FeatureMatching": "2.0",
            "ImageDetectionPrompt": "0.2",
            "ImageMatching": "2.0",
            "ImageSegmentationBox": "0.3",
            "IntrinsicsTransforming": "1.1",
            "NodalSfM": "2.0",
            "RelativePoseEstimating": "3.1",
            "ScenePreview": "2.0",
            "TracksBuilding": "1.0"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -400,
                0
            ],
            "inputs": {},
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ConvertDistortion_1": {
            "nodeType": "ConvertDistortion",
            "position": [
                1616,
                4
            ],
            "inputs": {
                "input": "{NodalSfM_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ConvertSfMFormat_1": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                1400,
                200
            ],
            "inputs": {
                "input": "{NodalSfM_1.output}",
                "fileExt": "sfm",
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
                2100,
                100
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
        "ExportAlembic_1": {
            "nodeType": "ExportAlembic",
            "position": [
                1807.0,
                -129.5
            ],
            "inputs": {
                "input": "{ExportImages_1.target}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportDistortion_1": {
            "nodeType": "ExportDistortion",
            "position": [
                1800,
                0
            ],
            "inputs": {
                "input": "{ConvertDistortion_1.output}",
                "exportLensGridsUndistorted": false
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ExportImages_1": {
            "nodeType": "ExportImages",
            "position": [
                1618.0,
                -132.5
            ],
            "inputs": {
                "input": "{IntrinsicsTransforming_1.input}",
                "target": "{IntrinsicsTransforming_1.output}",
                "outputFileType": "jpg",
                "namingMode": "keep"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "FeatureExtraction_1": {
            "nodeType": "FeatureExtraction",
            "position": [
                200,
                0
            ],
            "inputs": {
                "input": "{ImageSegmentationBox_1.input}",
                "masksFolder": "{ImageSegmentationBox_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
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
                "color": "#80766f"
            }
        },
        "ImageDetectionPrompt_1": {
            "nodeType": "ImageDetectionPrompt",
            "position": [
                -200,
                0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}"
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
                "input": "{FeatureExtraction_1.input}",
                "featuresFolders": [
                    "{FeatureExtraction_1.output}"
                ]
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ImageSegmentationBox_1": {
            "nodeType": "ImageSegmentationBox",
            "position": [
                0,
                0
            ],
            "inputs": {
                "input": "{ImageDetectionPrompt_1.input}",
                "bboxFolder": "{ImageDetectionPrompt_1.output}",
                "maskInvert": true,
                "keepFilename": true
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "IntrinsicsTransforming_1": {
            "nodeType": "IntrinsicsTransforming",
            "position": [
                1423.0,
                -132.5
            ],
            "inputs": {
                "input": "{NodalSfM_1.output}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "NodalSfM_1": {
            "nodeType": "NodalSfM",
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
                "color": "#80766f"
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
                "enforcePureRotation": true,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            },
            "internalInputs": {
                "color": "#80766f"
            }
        },
        "ScenePreview_1": {
            "nodeType": "ScenePreview",
            "position": [
                1799,
                185
            ],
            "inputs": {
                "cameras": "{ConvertSfMFormat_1.output}",
                "model": "{NodalSfM_1.output}",
                "undistortedImages": "{ExportImages_1.output}",
                "masks": "{ImageSegmentationBox_1.output}",
                "pointCloudParams": {
                    "particleSize": 0.001,
                    "particleColor": "Red"
                }
            },
            "internalInputs": {
                "color": "#4c594c"
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
                ]
            },
            "internalInputs": {
                "color": "#80766f"
            }
        }
    }
}