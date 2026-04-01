{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.1",
            "ConvertSfMFormat": "2.0",
            "CopyFiles": "1.3",
            "Depthmap": "5.1",
            "DepthMapFilter": "4.0",
            "FeatureExtraction": "1.3",
            "FeatureMatching": "2.0",
            "ImageDetectionPrompt": "1.0",
            "ImageMatching": "2.0",
            "ImageMatchingMultiSfM": "1.0",
            "ImageSegmentationBox": "1.0",
            "MeshFiltering": "3.0",
            "Meshing": "7.0",
            "PrepareDenseScene": "3.1",
            "SfMMerge": "3.0",
            "SfMTransform": "3.1",
            "SfMTriangulation": "1.0",
            "StructureFromMotion": "3.3",
            "Texturing": "6.0"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                0,
                300
            ],
            "inputs": {}
        },
        "CameraInit_2": {
            "nodeType": "CameraInit",
            "position": [
                0,
                600
            ],
            "inputs": {}
        },
        "ConvertSfMFormat_2": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                1000,
                750
            ],
            "inputs": {
                "input": "{StructureFromMotion_2.output}",
                "fileExt": "sfm",
                "structure": false,
                "observations": false,
                "surveys": false
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "ConvertSfMFormat_3": {
            "nodeType": "ConvertSfMFormat",
            "position": [
                1000,
                0
            ],
            "inputs": {
                "input": "{StructureFromMotion_1.output}",
                "fileExt": "sfm",
                "structure": false,
                "observations": false,
                "surveys": false
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "DepthMapFilter_3": {
            "nodeType": "DepthMapFilter",
            "position": [
                2400,
                400
            ],
            "inputs": {
                "input": "{DepthMap_3.input}",
                "depthMapsFolder": "{DepthMap_3.output}"
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        },
        "DepthMap_3": {
            "nodeType": "DepthMap",
            "position": [
                2200,
                400
            ],
            "inputs": {
                "input": "{PrepareDenseScene_3.input}",
                "imagesFolder": "{PrepareDenseScene_3.output}"
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        },
        "FeatureExtraction_1": {
            "nodeType": "FeatureExtraction",
            "position": [
                200,
                0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}"
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "FeatureExtraction_2": {
            "nodeType": "FeatureExtraction",
            "position": [
                200,
                750
            ],
            "inputs": {
                "input": "{CameraInit_2.output}"
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "FeatureExtraction_3": {
            "nodeType": "FeatureExtraction",
            "position": [
                600,
                300
            ],
            "inputs": {
                "input": "{ImageSegmentationBox_1.input}",
                "masksFolder": "{ImageSegmentationBox_1.output}"
            }
        },
        "FeatureExtraction_4": {
            "nodeType": "FeatureExtraction",
            "position": [
                600,
                600
            ],
            "inputs": {
                "input": "{ImageSegmentationBox_2.input}",
                "masksFolder": "{ImageSegmentationBox_2.output}"
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
                "color": "#507DD0"
            }
        },
        "FeatureMatching_2": {
            "nodeType": "FeatureMatching",
            "position": [
                600,
                750
            ],
            "inputs": {
                "input": "{ImageMatching_2.input}",
                "featuresFolders": "{ImageMatching_2.featuresFolders}",
                "imagePairsList": "{ImageMatching_2.output}",
                "describerTypes": "{FeatureExtraction_2.describerTypes}"
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "FeatureMatching_3": {
            "nodeType": "FeatureMatching",
            "position": [
                1000,
                400
            ],
            "inputs": {
                "input": "{ImageMatchingMultiSfM_1.outputCombinedSfM}",
                "featuresFolders": [
                    "{FeatureExtraction_3.output}",
                    "{FeatureExtraction_4.output}",
                    "{ImageMatchingMultiSfM_1.output}"
                ],
                "imagePairsList": "{ImageMatchingMultiSfM_1.output}"
            },
            "internalInputs": {
                "color": "#E35C03"
            }
        },
        "FeatureMatching_4": {
            "nodeType": "FeatureMatching",
            "position": [
                1000,
                200
            ],
            "inputs": {
                "input": "{ImageMatching_3.input}",
                "featuresFolders": "{ImageMatching_3.featuresFolders}",
                "imagePairsList": "{ImageMatching_3.output}"
            }
        },
        "FeatureMatching_5": {
            "nodeType": "FeatureMatching",
            "position": [
                1000,
                600
            ],
            "inputs": {
                "input": "{ImageMatching_4.input}",
                "featuresFolders": "{ImageMatching_4.featuresFolders}",
                "imagePairsList": "{ImageMatching_4.output}"
            }
        },
        "ImageDetectionPrompt_1": {
            "nodeType": "ImageDetectionPrompt",
            "position": [
                200,
                300
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "prompt": "main",
                "synonyms": "",
                "forceDetection": true,
                "thresholdDetection": 0.3
            }
        },
        "ImageDetectionPrompt_2": {
            "nodeType": "ImageDetectionPrompt",
            "position": [
                200,
                600
            ],
            "inputs": {
                "input": "{CameraInit_2.output}",
                "prompt": "main",
                "synonyms": "",
                "forceDetection": true,
                "thresholdDetection": 0.3
            }
        },
        "ImageMatchingMultiSfM_1": {
            "nodeType": "ImageMatchingMultiSfM",
            "position": [
                800,
                400
            ],
            "inputs": {
                "input": "{FeatureExtraction_3.input}",
                "inputB": "{FeatureExtraction_4.input}",
                "featuresFolders": [
                    "{FeatureExtraction_3.output}",
                    "{FeatureExtraction_4.output}"
                ],
                "method": "VocabularyTree",
                "matchingMode": "a/b"
            },
            "internalInputs": {
                "color": "#E35C03"
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
                "color": "#507DD0"
            }
        },
        "ImageMatching_2": {
            "nodeType": "ImageMatching",
            "position": [
                400,
                750
            ],
            "inputs": {
                "input": "{FeatureExtraction_2.input}",
                "featuresFolders": [
                    "{FeatureExtraction_2.output}"
                ]
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "ImageMatching_3": {
            "nodeType": "ImageMatching",
            "position": [
                800,
                200
            ],
            "inputs": {
                "input": "{FeatureExtraction_3.input}",
                "featuresFolders": [
                    "{FeatureExtraction_3.output}"
                ]
            }
        },
        "ImageMatching_4": {
            "nodeType": "ImageMatching",
            "position": [
                800,
                600
            ],
            "inputs": {
                "input": "{FeatureExtraction_4.input}",
                "featuresFolders": [
                    "{FeatureExtraction_4.output}"
                ]
            }
        },
        "ImageSegmentationBox_1": {
            "nodeType": "ImageSegmentationBox",
            "position": [
                400,
                300
            ],
            "inputs": {
                "input": "{ImageDetectionPrompt_1.input}",
                "bboxFolder": "{ImageDetectionPrompt_1.output}",
                "keepFilename": true,
                "extensionOut": "png"
            }
        },
        "ImageSegmentationBox_2": {
            "nodeType": "ImageSegmentationBox",
            "position": [
                400,
                600
            ],
            "inputs": {
                "input": "{ImageDetectionPrompt_2.input}",
                "bboxFolder": "{ImageDetectionPrompt_2.output}",
                "keepFilename": true,
                "extensionOut": "png"
            }
        },
        "MeshFiltering_3": {
            "nodeType": "MeshFiltering",
            "position": [
                2800,
                400
            ],
            "inputs": {
                "inputMesh": "{Meshing_3.outputMesh}"
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        },
        "Meshing_3": {
            "nodeType": "Meshing",
            "position": [
                2600,
                400
            ],
            "inputs": {
                "input": "{DepthMapFilter_3.input}",
                "depthMapsFolder": "{DepthMapFilter_3.output}"
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        },
        "PrepareDenseScene_3": {
            "nodeType": "PrepareDenseScene",
            "position": [
                2000,
                400
            ],
            "inputs": {
                "input": "{SfMTransform_1.output}",
                "masksFolders": [
                    "{ImageSegmentationBox_1.output}",
                    "{ImageSegmentationBox_2.output}"
                ]
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        },
        "CopyFiles_1": {
            "nodeType": "CopyFiles",
            "position": [
                3200,
                400
            ],
            "inputs": {
                "inputFiles": [
                    "{Texturing_3.outputMesh}",
                    "{Texturing_3.outputMaterial}",
                    "{Texturing_3.outputTextures}",
                    "{SfMTransform_1.output}"
                ]
            }
        },
        "SfMMerge_1": {
            "nodeType": "SfMMerge",
            "position": [
                1400,
                400
            ],
            "inputs": {
                "inputs": [
                    "{SfMTriangulation_3.output}",
                    "{SfMTriangulation_2.output}"
                ],
                "method": "from_landmarks",
                "matchesFolders": [
                    "{FeatureMatching_3.output}"
                ]
            },
            "internalInputs": {
                "invalidation": "fix landmarks",
                "color": "#E35C03"
            }
        },
        "SfMTransform_1": {
            "nodeType": "SfMTransform",
            "position": [
                1800,
                400
            ],
            "inputs": {
                "input": "{SfMTriangulation_4.output}"
            }
        },
        "SfMTriangulation_2": {
            "nodeType": "SfMTriangulation",
            "position": [
                1200,
                200
            ],
            "inputs": {
                "input": "{ConvertSfMFormat_3.output}",
                "featuresFolders": "{FeatureMatching_4.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_4.output}"
                ]
            }
        },
        "SfMTriangulation_3": {
            "nodeType": "SfMTriangulation",
            "position": [
                1200,
                600
            ],
            "inputs": {
                "input": "{ConvertSfMFormat_2.output}",
                "featuresFolders": "{FeatureMatching_5.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_5.output}"
                ]
            }
        },
        "SfMTriangulation_4": {
            "nodeType": "SfMTriangulation",
            "position": [
                1600,
                400
            ],
            "inputs": {
                "input": "{SfMMerge_1.output}",
                "featuresFolders": "{FeatureMatching_5.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_3.output}",
                    "{FeatureMatching_5.output}",
                    "{FeatureMatching_4.output}"
                ]
            }
        },
        "StructureFromMotion_1": {
            "nodeType": "StructureFromMotion",
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
                "describerTypes": "{FeatureMatching_1.describerTypes}"
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "StructureFromMotion_2": {
            "nodeType": "StructureFromMotion",
            "position": [
                800,
                750
            ],
            "inputs": {
                "input": "{FeatureMatching_2.input}",
                "featuresFolders": "{FeatureMatching_2.featuresFolders}",
                "matchesFolders": [
                    "{FeatureMatching_2.output}"
                ],
                "describerTypes": "{FeatureMatching_2.describerTypes}"
            },
            "internalInputs": {
                "color": "#507DD0"
            }
        },
        "Texturing_3": {
            "nodeType": "Texturing",
            "position": [
                3000,
                400
            ],
            "inputs": {
                "input": "{Meshing_3.output}",
                "imagesFolder": "{DepthMap_3.imagesFolder}",
                "inputMesh": "{MeshFiltering_3.outputMesh}"
            },
            "internalInputs": {
                "color": "#4D3E5C"
            }
        }
    }
}
