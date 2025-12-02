{
    "header": {
        "releaseVersion": "2026.1.0+develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "PhotometricStereo": "1.0",
            "PrepareDenseScene": "3.1",
            "SfMFilter": "1.0",
            "SfMMerge": "3.0",
            "SfMTransfer": "2.1",
            "Texturing": "6.0"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -122,
                -120
            ],
            "inputs": {}
        },
        "PhotometricStereo_1": {
            "nodeType": "PhotometricStereo",
            "position": [
                595,
                -653
            ],
            "inputs": {
                "inputPath": "{SfMFilter_4.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_2": {
            "nodeType": "PhotometricStereo",
            "position": [
                583.5,
                -405.0
            ],
            "inputs": {
                "inputPath": "{SfMFilter_5.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_3": {
            "nodeType": "PhotometricStereo",
            "position": [
                622.5,
                108.0
            ],
            "inputs": {
                "inputPath": "{SfMFilter_2.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_4": {
            "nodeType": "PhotometricStereo",
            "position": [
                621.5,
                339.0
            ],
            "inputs": {
                "inputPath": "{SfMFilter_3.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_5": {
            "nodeType": "PhotometricStereo",
            "position": [
                1019,
                -647
            ],
            "inputs": {
                "inputPath": "{SfMFilter_6.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_6": {
            "nodeType": "PhotometricStereo",
            "position": [
                1036.5,
                -423.0
            ],
            "inputs": {
                "inputPath": "{SfMFilter_7.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PhotometricStereo_7": {
            "nodeType": "PhotometricStereo",
            "position": [
                1080.5,
                144.0
            ],
            "inputs": {
                "inputPath": "{SfMFilter_8.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks"
            }
        },
        "PrepareDenseScene_7": {
            "nodeType": "PrepareDenseScene",
            "position": [
                2085,
                -310
            ],
            "parallelization": {
                "blockSize": 40,
                "size": 7,
                "split": 1
            },
            "uid": "a91a496f49031aa7d6ead7f01f7dd1508667651e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMTransfer_2.output}"
            }
        },
        "SfMFilter_1": {
            "nodeType": "SfMFilter",
            "position": [
                116,
                -120
            ],
            "inputs": {
                "inputFile": "{CameraInit_1.output}",
                "fileMatchingPattern": ".*led_19.*"
            }
        },
        "SfMFilter_2": {
            "nodeType": "SfMFilter",
            "position": [
                416,
                145
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_02.*"
            }
        },
        "SfMFilter_3": {
            "nodeType": "SfMFilter",
            "position": [
                405,
                375
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_03.*"
            }
        },
        "SfMFilter_4": {
            "nodeType": "SfMFilter",
            "position": [
                394.5,
                -638.0
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_00.*"
            }
        },
        "SfMFilter_5": {
            "nodeType": "SfMFilter",
            "position": [
                383.5,
                -408.0
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_01.*"
            }
        },
        "SfMFilter_6": {
            "nodeType": "SfMFilter",
            "position": [
                828.5,
                -637.0
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_04.*"
            }
        },
        "SfMFilter_7": {
            "nodeType": "SfMFilter",
            "position": [
                819.5,
                -391.0
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_05.*"
            }
        },
        "SfMFilter_8": {
            "nodeType": "SfMFilter",
            "position": [
                883.5,
                135.0
            ],
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_06.*",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputSfMData_selected": "{nodeCacheFolder}/selectedSfmData.sfm",
                "outputSfMData_unselected": "{nodeCacheFolder}/unselectedSfmData.sfm"
            }
        },
        "SfMMerge_3": {
            "nodeType": "SfMMerge",
            "position": [
                1598,
                -295
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 7,
                "split": 1
            },
            "uid": "1951f9866218fc218aac1baf903164bc5f98b333",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputs": [
                    "{PhotometricStereo_1.outputSfmDataAlbedo}",
                    "{PhotometricStereo_5.outputSfmDataAlbedo}",
                    "{PhotometricStereo_7.outputSfmDataAlbedo}",
                    "{PhotometricStereo_3.outputSfmDataAlbedo}",
                    "{PhotometricStereo_6.outputSfmDataAlbedo}",
                    "{PhotometricStereo_2.outputSfmDataAlbedo}",
                    "{PhotometricStereo_4.outputSfmDataAlbedo}"
                ]
            }
        },
        "SfMTransfer_1": {
            "nodeType": "SfMTransfer",
            "position": [
                953,
                -152
            ],
            "inputs": {
                "input": "{SfMFilter_1.outputSfMData_selected}",
                "reference": "/path/to/poses",
                "method": "from_filepath"
            }
        },
		"DepthMapRename_1": {
			"nodeType": "DepthMapRename",
			"position" : [
				1836,
                20
			],
			"parallelization": {
                "blockSize": 0,
                "size": 2142,
                "split": 1
            },
            "uid": "696431f45f19245c8a9a02e94363445f53f67e65",
            "inputs": {
                "input": "{SfMTransfer_1.output}",
                "depthMapFolder": "/path/to/depthMapFolder",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "nodeDefaultLogLevel": "info",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}"
            }
		},
        "SfMTransfer_2": {
            "nodeType": "SfMTransfer",
            "position": [
                1836,
                -181
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 7,
                "split": 1
            },
            "uid": "15a09d284260835737b09320ad135bc8fa6a66b1",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMMerge_3.output}",
                "reference": "{SfMTransfer_1.output}"
            }
        },
        "Texturing_6": {
            "nodeType": "Texturing",
            "position": [
                2513,
                -204
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "afaec7600d08984ef40280d5139facf8c5440067",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMTransfer_2.output}",
                "imagesFolder": "{PrepareDenseScene_7.output}",
				"depthMapFolder": "{DepthMapRename_1.output}",
                "inputMesh": "/path/to/mesh",
                "textureSide": 4096,
                "downscale": 1,
                "colorMapping": {
                    "enable": true,
                    "colorMappingFileType": "png"
                },
                "visibilityRemappingMethod": "DepthMap",
                "verboseLevel": "debug"
            }
        }
    }
}