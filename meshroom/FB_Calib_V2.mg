{
    "header": {
        "releaseVersion": "2026.1.0+develop",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "LightingCalibration": "1.0",
            "SphereDetection": "1.0"
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
            "inputs": {}
        },
        "CameraInit_2": {
            "nodeType": "CameraInit",
            "position": [
                0,
                160
            ],
            "inputs": {}
        },
        "CameraInit_3": {
            "nodeType": "CameraInit",
            "position": [
                0,
                320
            ],
            "inputs": {}
        },
        "CameraInit_4": {
            "nodeType": "CameraInit",
            "position": [
                0,
                480
            ],
            "inputs": {}
        },
        "CameraInit_5": {
            "nodeType": "CameraInit",
            "position": [
                660,
                10
            ],
            "inputs": {}
        },
        "CameraInit_6": {
            "nodeType": "CameraInit",
            "position": [
                660,
                170
            ],
            "inputs": {}
        },
        "CameraInit_7": {
            "nodeType": "CameraInit",
            "position": [
                660,
                330
            ],
            "inputs": {}
        },
        "CameraInit_8": {
            "nodeType": "CameraInit",
            "position": [
                660,
                490
            ],
            "inputs": {}
        },
        "LightingCalibration_1": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                0
            ],
            "inputs": {
                "inputPath": "{SphereDetection_1.input}",
                "inputDetection": "{SphereDetection_1.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_2": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                160
            ],
            "inputs": {
                "inputPath": "{SphereDetection_2.input}",
                "inputDetection": "{SphereDetection_2.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_3": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                320
            ],
            "inputs": {
                "inputPath": "{SphereDetection_3.input}",
                "inputDetection": "{SphereDetection_3.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_4": {
            "nodeType": "LightingCalibration",
            "position": [
                400,
                480
            ],
            "inputs": {
                "inputPath": "{SphereDetection_4.input}",
                "inputDetection": "{SphereDetection_4.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_5": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                10
            ],
            "inputs": {
                "inputPath": "{SphereDetection_5.input}",
                "inputDetection": "{SphereDetection_5.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_6": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                170
            ],
            "inputs": {
                "inputPath": "{SphereDetection_6.input}",
                "inputDetection": "{SphereDetection_6.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_7": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                330
            ],
            "inputs": {
                "inputPath": "{SphereDetection_7.input}",
                "inputDetection": "{SphereDetection_7.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "LightingCalibration_8": {
            "nodeType": "LightingCalibration",
            "position": [
                1060,
                490
            ],
            "inputs": {
                "inputPath": "{SphereDetection_8.input}",
                "inputDetection": "{SphereDetection_8.output}",
                "saveAsModel": true,
                "method": "whiteSphere"
            }
        },
        "SphereDetection_1": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}"
            }
        },
        "SphereDetection_2": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                160
            ],
            "inputs": {
                "input": "{CameraInit_2.output}"
            }
        },
        "SphereDetection_3": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                320
            ],
            "inputs": {
                "input": "{CameraInit_3.output}"
            }
        },
        "SphereDetection_4": {
            "nodeType": "SphereDetection",
            "position": [
                200,
                480
            ],
            "inputs": {
                "input": "{CameraInit_4.output}"
            }
        },
        "SphereDetection_5": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                10
            ],
            "inputs": {
                "input": "{CameraInit_5.output}"
            }
        },
        "SphereDetection_6": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                170
            ],
            "inputs": {
                "input": "{CameraInit_6.output}"
            }
        },
        "SphereDetection_7": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                330
            ],
            "inputs": {
                "input": "{CameraInit_7.output}"
            }
        },
        "SphereDetection_8": {
            "nodeType": "SphereDetection",
            "position": [
                860,
                490
            ],
            "inputs": {
                "input": "{CameraInit_8.output}"
            }
        }
    }
}