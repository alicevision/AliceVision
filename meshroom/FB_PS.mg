{
    "header": {
        "releaseVersion": "2025.1.0-develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "PhotometricStereo": "1.0",
            "PrepareDenseScene": "3.1",
            "SfMFilter": "1.0",
            "SfMMerge": "3.0",
            "SfMTransfer": "2.1",
            "Texturing": "6.0"
        }
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -122,
                -120
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 0,
                "split": 1
            },
            "uid": "4c471e04d91d618d3dddf7990cb9b81c599a7700",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "viewpoints": [],
                "intrinsics": [],
                "sensorDatabase": "${ALICEVISION_SENSOR_DB}",
                "lensCorrectionProfileInfo": "${ALICEVISION_LENS_PROFILE_INFO}",
                "lensCorrectionProfileSearchIgnoreCameraModel": true,
                "defaultFieldOfView": 45.0,
                "groupCameraFallback": "folder",
                "rawColorInterpretation": "LibRawWhiteBalancing",
                "colorProfileDatabase": "${ALICEVISION_COLOR_PROFILE_DB}",
                "errorOnMissingColorProfile": true,
                "viewIdMethod": "metadata",
                "viewIdRegex": ".*?(\\d+)",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/cameraInit.sfm"
            }
        },
        "PhotometricStereo_1": {
            "nodeType": "PhotometricStereo",
            "position": [
                605,
                378
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "c9709d61581e940af4c0f8469e3425d0841c81b7",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_3.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_2": {
            "nodeType": "PhotometricStereo",
            "position": [
                583.5,
                -405.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "da1f77c463ea699752f1b8bc409ba4a90af78c54",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_5.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_3": {
            "nodeType": "PhotometricStereo",
            "position": [
                588.5,
                -653.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "e625fb1898f175b05e72bfd1e4be72613e9d0410",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_4.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_4": {
            "nodeType": "PhotometricStereo",
            "position": [
                1019.5,
                -388.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "a452f39b6da5594ad5f333dcca74c29cbd1a4bce",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_7.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_5": {
            "nodeType": "PhotometricStereo",
            "position": [
                610,
                130
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "abbb2c152cf4095634686788895a60bdc162a749",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_2.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_6": {
            "nodeType": "PhotometricStereo",
            "position": [
                1024.5,
                -636.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "faa92d0960d6ceba397b79d54ec1704034ee31fe",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_6.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_7": {
            "nodeType": "PhotometricStereo",
            "position": [
                1072.5,
                368.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "0432a3a1e24ed514cf53c55bafca2d1ca26dbf44",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_9.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
            }
        },
        "PhotometricStereo_8": {
            "nodeType": "PhotometricStereo",
            "position": [
                1077.5,
                120.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "50b72c66b9f85f41ef4131c35f8315af39316e50",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SfMFilter_8.outputSfMData_selected}",
                "pathToJSONLightFile": "/path/to/lightfile",
                "maskPath": "/path/to/masks",
                "SHOrder": "0",
                "removeAmbient": false,
                "isRobust": false,
                "downscale": 1,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputPath": "{nodeCacheFolder}",
                "outputSfmDataAlbedo": "{nodeCacheFolder}/albedoMaps.sfm",
                "outputSfmDataNormal": "{nodeCacheFolder}/normalMaps.sfm",
                "outputSfmDataNormalPNG": "{nodeCacheFolder}/normalMapsPNG.sfm",
                "normals": "{nodeCacheFolder}/<POSE_ID>_normals.exr",
                "normalsPNG": "{nodeCacheFolder}/<POSE_ID>_normals.png",
                "normalsWorld": "{nodeCacheFolder}/<POSE_ID>_normals_w.exr",
                "albedo": "{nodeCacheFolder}/<POSE_ID>_albedo.png"
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
                "size": 8,
                "split": 1
            },
            "uid": "1c037f95c83ccee5be2ea8516aee92561ba092a8",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMTransfer_2.output}",
                "imagesFolders": [],
                "masksFolders": [],
                "maskExtension": "png",
                "outputFileType": "exr",
                "saveMetadata": true,
                "saveMatricesTxtFiles": false,
                "evCorrection": false,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}",
                "undistorted": "{nodeCacheFolder}/<VIEW_ID>.{outputFileTypeValue}"
            }
        },
        "SfMFilter_1": {
            "nodeType": "SfMFilter",
            "position": [
                116,
                -120
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "05c1435d4f6c006e479efc72cb882eee6d115763",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{CameraInit_1.output}",
                "fileMatchingPattern": ".*led_19.*",
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
        "SfMFilter_2": {
            "nodeType": "SfMFilter",
            "position": [
                416,
                145
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "6932a1c6b595573624ea2e9183f7d56bbb6d06de",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_02.*",
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
        "SfMFilter_3": {
            "nodeType": "SfMFilter",
            "position": [
                405,
                375
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "dc7a9e3cf690833d2d85f9a8501ea10b008fe53c",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_03.*",
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
        "SfMFilter_4": {
            "nodeType": "SfMFilter",
            "position": [
                394.5,
                -638.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "12b1660307541809b58c9ba351ff048326f8c180",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_00.*",
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
        "SfMFilter_5": {
            "nodeType": "SfMFilter",
            "position": [
                383.5,
                -408.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "c6c1d71d9bef9a7b972e09bb459fad10f16d8a82",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_01.*",
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
        "SfMFilter_6": {
            "nodeType": "SfMFilter",
            "position": [
                830.5,
                -621.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "67bcad3913a98cc7ca882d3191a43c76e9992d8a",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_04.*",
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
        "SfMFilter_7": {
            "nodeType": "SfMFilter",
            "position": [
                819.5,
                -391.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "3369f8c6bca3fc21371a3b001a03477e17fea688",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_05.*",
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
        "SfMFilter_8": {
            "nodeType": "SfMFilter",
            "position": [
                883.5,
                135.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "78685b339c516bf0e1e0d3c10cf087a66d26201f",
            "internalFolder": "{cache}/{nodeType}/{uid}",
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
        "SfMFilter_9": {
            "nodeType": "SfMFilter",
            "position": [
                872.5,
                365.0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "06876475a443e478fe923bfd684d8edcdb4054d9",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputFile": "{SfMFilter_1.outputSfMData_unselected}",
                "fileMatchingPattern": ".*cam_07.*",
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
                "size": 8,
                "split": 1
            },
            "uid": "33e4f52322f5583174bcbc22d7e825989c091017",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputs": [
                    "{PhotometricStereo_1.outputSfmDataAlbedo}",
                    "{PhotometricStereo_5.outputSfmDataAlbedo}",
                    "{PhotometricStereo_8.outputSfmDataAlbedo}",
                    "{PhotometricStereo_7.outputSfmDataAlbedo}",
                    "{PhotometricStereo_3.outputSfmDataAlbedo}",
                    "{PhotometricStereo_6.outputSfmDataAlbedo}",
                    "{PhotometricStereo_2.outputSfmDataAlbedo}",
                    "{PhotometricStereo_4.outputSfmDataAlbedo}"
                ],
                "method": "simple_copy",
                "matchesFolders": [],
                "describerTypes": [
                    "dspsift"
                ],
                "fileExt": "abc",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/sfmData.{fileExtValue}"
            }
        },
        "SfMTransfer_1": {
            "nodeType": "SfMTransfer",
            "position": [
                953,
                -152
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "372c4d9438e0cb274ae8649e2d23062584e54cb5",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMFilter_1.outputSfMData_selected}",
                "reference": "/path/to/poses",
                "method": "from_filepath",
                "fileMatchingPattern": ".*\\/(.*?)\\.\\w{3}",
                "metadataMatchingList": [],
                "transferPoses": true,
                "transferIntrinsics": true,
                "transferLandmarks": true,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/selectedSfmData.abc",
                "outputViewsAndPoses": "{nodeCacheFolder}/cameras.sfm"
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
                "size": 8,
                "split": 1
            },
            "uid": "fa567c89c5957d423520da55a3d706aa95b44bf0",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMMerge_3.output}",
                "reference": "{SfMTransfer_1.output}",
                "method": "from_viewid",
                "fileMatchingPattern": ".*\\/(.*?)\\.\\w{3}",
                "metadataMatchingList": [],
                "transferPoses": true,
                "transferIntrinsics": true,
                "transferLandmarks": true,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/sfmData.abc",
                "outputViewsAndPoses": "{nodeCacheFolder}/cameras.sfm"
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
            "uid": "d38d70fdbbd368162f5b3ee6e677afb50657b874",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{SfMTransfer_2.output}",
                "imagesFolder": "{PrepareDenseScene_7.output}",
                "normalsFolder": "",
                "inputMesh": "/path/to/mesh",
                "inputRefMesh": "",
                "textureSide": 4096,
                "downscale": 1,
                "outputMeshFileType": "obj",
                "colorMapping": {
                    "enable": true,
                    "colorMappingFileType": "png"
                },
                "bumpMapping": {
                    "enable": true,
                    "bumpType": "Normal",
                    "normalFileType": "exr",
                    "heightFileType": "exr"
                },
                "displacementMapping": {
                    "enable": true,
                    "displacementMappingFileType": "exr"
                },
                "unwrapMethod": "Basic",
                "useUDIM": true,
                "fillHoles": false,
                "padding": 5,
                "multiBandDownscale": 4,
                "multiBandNbContrib": {
                    "high": 1,
                    "midHigh": 5,
                    "midLow": 10,
                    "low": 0
                },
                "useScore": true,
                "bestScoreThreshold": 0.1,
                "angleHardThreshold": 90.0,
                "workingColorSpace": "sRGB",
                "outputColorSpace": "AUTO",
                "correctEV": true,
                "forceVisibleByAllVertices": false,
                "flipNormals": false,
                "visibilityRemappingMethod": "MeshItself",
                "subdivisionTargetRatio": 0.8,
                "verboseLevel": "debug"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}",
                "outputMesh": "{nodeCacheFolder}/texturedMesh.{outputMeshFileTypeValue}",
                "outputMaterial": "{nodeCacheFolder}/texturedMesh.mtl",
                "outputTextures": "{nodeCacheFolder}/texture_*.png"
            }
        }
    }
}