{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.1",
            "CheckerboardDetection": "2.0",
            "DistortionCalibration": "6.1",
            "ExportDistortion": "2.0",
            "CopyFiles": "1.3"
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
        "CheckerboardDetection_1": {
            "nodeType": "CheckerboardDetection",
            "position": [
                200,
                0
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "useNestedGrids": true,
                "exportDebugImages": true
            }
        },
        "DistortionCalibration_1": {
            "nodeType": "DistortionCalibration",
            "position": [
                400,
                0
            ],
            "inputs": {
                "input": "{CheckerboardDetection_1.input}",
                "checkerboards": "{CheckerboardDetection_1.output}"
            }
        },
        "ExportDistortion_1": {
            "nodeType": "ExportDistortion",
            "position": [
                600,
                0
            ],
            "inputs": {
                "input": "{DistortionCalibration_1.output}"
            }
        },
        "CopyFiles_1": {
            "nodeType": "CopyFiles",
            "position": [
                800,
                0
            ],
            "inputs": {
                "inputFiles": [
                    "{ExportDistortion_1.output}"
                ]
            }
        }
    }
}