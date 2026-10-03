{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.1",
            "FeatureExtraction": "1.3",
            "FeatureMatching": "2.1",
            "ImageMatching": "2.0",
            "RelativePoseEstimating": "3.2",
            "SfMBootStrapping": "4.3",
            "SfMComparing": "1.0",
            "SfMExpanding": "2.7",
            "TracksBuilding": "1.0"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                0,
                0
            ],
            "inputs": {
                "viewIdMethod": "filename"
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
                "minInliers": 100,
                "randomSeed": 1,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_10": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                1440
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 10,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_2": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                160
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 2,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_3": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                320
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 3,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_4": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                480
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 4,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_5": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                640
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 5,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_6": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                800
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 6,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_7": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                960
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 7,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_8": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                1120
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 8,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
            }
        },
        "RelativePoseEstimating_9": {
            "nodeType": "RelativePoseEstimating",
            "position": [
                1000,
                1280
            ],
            "inputs": {
                "input": "{TracksBuilding_1.input}",
                "tracksFilename": "{TracksBuilding_1.output}",
                "minInliers": 100,
                "randomSeed": 9,
                "imagePairsList": "{FeatureMatching_1.imagePairsList}"
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
            }
        },
        "SfMBootStrapping_10": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                1440
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_10.input}",
                "tracksFilename": "{RelativePoseEstimating_10.tracksFilename}",
                "pairs": "{RelativePoseEstimating_10.output}"
            }
        },
        "SfMBootStrapping_2": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                160
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_2.input}",
                "tracksFilename": "{RelativePoseEstimating_2.tracksFilename}",
                "pairs": "{RelativePoseEstimating_2.output}"
            }
        },
        "SfMBootStrapping_3": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                320
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_3.input}",
                "tracksFilename": "{RelativePoseEstimating_3.tracksFilename}",
                "pairs": "{RelativePoseEstimating_3.output}"
            }
        },
        "SfMBootStrapping_4": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                480
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_4.input}",
                "tracksFilename": "{RelativePoseEstimating_4.tracksFilename}",
                "pairs": "{RelativePoseEstimating_4.output}"
            }
        },
        "SfMBootStrapping_5": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                640
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_5.input}",
                "tracksFilename": "{RelativePoseEstimating_5.tracksFilename}",
                "pairs": "{RelativePoseEstimating_5.output}"
            }
        },
        "SfMBootStrapping_6": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                800
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_6.input}",
                "tracksFilename": "{RelativePoseEstimating_6.tracksFilename}",
                "pairs": "{RelativePoseEstimating_6.output}"
            }
        },
        "SfMBootStrapping_7": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                960
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_7.input}",
                "tracksFilename": "{RelativePoseEstimating_7.tracksFilename}",
                "pairs": "{RelativePoseEstimating_7.output}"
            }
        },
        "SfMBootStrapping_8": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                1120
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_8.input}",
                "tracksFilename": "{RelativePoseEstimating_8.tracksFilename}",
                "pairs": "{RelativePoseEstimating_8.output}"
            }
        },
        "SfMBootStrapping_9": {
            "nodeType": "SfMBootStrapping",
            "position": [
                1200,
                1280
            ],
            "inputs": {
                "input": "{RelativePoseEstimating_9.input}",
                "tracksFilename": "{RelativePoseEstimating_9.tracksFilename}",
                "pairs": "{RelativePoseEstimating_9.output}"
            }
        },
        "SfMComparing_1": {
            "nodeType": "SfMComparing",
            "position": [
                1600,
                720
            ],
            "inputs": {
                "input": [
                    "{SfMExpanding_1.output}",
                    "{SfMExpanding_2.output}",
                    "{SfMExpanding_3.output}",
                    "{SfMExpanding_4.output}",
                    "{SfMExpanding_5.output}",
                    "{SfMExpanding_6.output}",
                    "{SfMExpanding_7.output}",
                    "{SfMExpanding_8.output}",
                    "{SfMExpanding_9.output}",
                    "{SfMExpanding_10.output}"
                ]
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
                "randomSeed": 1
            }
        },
        "SfMExpanding_10": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                1440
            ],
            "inputs": {
                "input": "{SfMBootStrapping_10.output}",
                "tracksFilename": "{SfMBootStrapping_10.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_10.meshFilename}",
                "randomSeed": 10
            }
        },
        "SfMExpanding_2": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                160
            ],
            "inputs": {
                "input": "{SfMBootStrapping_2.output}",
                "tracksFilename": "{SfMBootStrapping_2.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_2.meshFilename}",
                "randomSeed": 2
            }
        },
        "SfMExpanding_3": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                320
            ],
            "inputs": {
                "input": "{SfMBootStrapping_3.output}",
                "tracksFilename": "{SfMBootStrapping_3.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_3.meshFilename}",
                "randomSeed": 3
            }
        },
        "SfMExpanding_4": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                480
            ],
            "inputs": {
                "input": "{SfMBootStrapping_4.output}",
                "tracksFilename": "{SfMBootStrapping_4.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_4.meshFilename}",
                "randomSeed": 4
            }
        },
        "SfMExpanding_5": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                640
            ],
            "inputs": {
                "input": "{SfMBootStrapping_5.output}",
                "tracksFilename": "{SfMBootStrapping_5.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_5.meshFilename}",
                "randomSeed": 5
            }
        },
        "SfMExpanding_6": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                800
            ],
            "inputs": {
                "input": "{SfMBootStrapping_6.output}",
                "tracksFilename": "{SfMBootStrapping_6.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_6.meshFilename}",
                "randomSeed": 6
            }
        },
        "SfMExpanding_7": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                960
            ],
            "inputs": {
                "input": "{SfMBootStrapping_7.output}",
                "tracksFilename": "{SfMBootStrapping_7.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_7.meshFilename}",
                "randomSeed": 7
            }
        },
        "SfMExpanding_8": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                1120
            ],
            "inputs": {
                "input": "{SfMBootStrapping_8.output}",
                "tracksFilename": "{SfMBootStrapping_8.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_8.meshFilename}",
                "randomSeed": 8
            }
        },
        "SfMExpanding_9": {
            "nodeType": "SfMExpanding",
            "position": [
                1400,
                1280
            ],
            "inputs": {
                "input": "{SfMBootStrapping_9.output}",
                "tracksFilename": "{SfMBootStrapping_9.tracksFilename}",
                "meshFilename": "{SfMBootStrapping_9.meshFilename}",
                "randomSeed": 9
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
                "describerTypes": "{FeatureMatching_1.describerTypes}"
            }
        }
    }
}