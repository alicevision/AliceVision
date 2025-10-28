{
    "header": {
        "releaseVersion": "2025.1.0-develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "LightingCalibration": "1.0",
            "SphereDetection": "1.0"
        }
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                0,
                0
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
        "CameraInit_2": {
            "nodeType": "CameraInit",
            "position": [
                0,
                160
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
        "CameraInit_3": {
            "nodeType": "CameraInit",
            "position": [
                0,
                320
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
        "CameraInit_4": {
            "nodeType": "CameraInit",
            "position": [
                0,
                480
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
        "CameraInit_5": {
            "nodeType": "CameraInit",
            "position": [
                660,
                10
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
        "CameraInit_6": {
            "nodeType": "CameraInit",
            "position": [
                660,
                170
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
        "CameraInit_7": {
            "nodeType": "CameraInit",
            "position": [
                660,
                330
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
        "CameraInit_8": {
            "nodeType": "CameraInit",
            "position": [
                660,
                490
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
        "LightingCalibration_1": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_1.input}",
                "inputDetection": "{SphereDetection_1.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_2": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                160
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_2.input}",
                "inputDetection": "{SphereDetection_2.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_3": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                320
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_3.input}",
                "inputDetection": "{SphereDetection_3.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_4": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                480
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_4.input}",
                "inputDetection": "{SphereDetection_4.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_5": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                10
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_5.input}",
                "inputDetection": "{SphereDetection_5.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_6": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                170
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_6.input}",
                "inputDetection": "{SphereDetection_6.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_7": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                330
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_7.input}",
                "inputDetection": "{SphereDetection_7.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "LightingCalibration_8": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                490
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "939a5543efd97e70307a1a5b9814a51301d87d3d",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "inputPath": "{SphereDetection_8.input}",
                "inputDetection": "{SphereDetection_8.output}",
                "saveAsModel": false,
                "ellipticEstimation": false,
                "method": "brightestPoint",
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "outputFile": "{nodeCacheFolder}/lights.json",
                "lightingEstimationVisualization": "{nodeCacheFolder}/<FILESTEM>_{methodValue}.png"
            }
        },
        "SphereDetection_1": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                0
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_1.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_2": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                160
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_2.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_3": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                320
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_3.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_4": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                480
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_4.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_5": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                10
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_5.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_6": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                170
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_6.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_7": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                330
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_7.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        },
        "SphereDetection_8": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                490
            ],
            "parallelization": {
                "blockSize": 0,
                "size": 1,
                "split": 1
            },
            "uid": "4252c728039b776824bbe769d34bcc8bef200e6e",
            "internalFolder": "{cache}/{nodeType}/{uid}",
            "inputs": {
                "input": "{CameraInit_8.output}",
                "modelPath": "${ALICEVISION_SPHERE_DETECTION_MODEL}",
                "autoDetect": false,
                "minScore": 0.0,
                "sphereCenter": {
                    "x": 0.0,
                    "y": 0.0
                },
                "sphereRadius": 500.0,
                "verboseLevel": "info"
            },
            "internalInputs": {
                "invalidation": "",
                "comment": "",
                "label": "",
                "color": ""
            },
            "outputs": {
                "output": "{nodeCacheFolder}/detection.json"
            }
        }
    }
}