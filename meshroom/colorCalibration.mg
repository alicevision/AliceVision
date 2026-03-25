{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.1",
            "ColorCheckerCorrection": "2.0",
            "ColorCheckerDetection": "2.0",
            "CopyFiles": "1.3"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -309,
                -26
            ],
            "inputs": {}
        },
        "ColorCheckerCorrection_1": {
            "nodeType": "ColorCheckerCorrection",
            "position": [
                87,
                -24
            ],
            "inputs": {
                "inputData": "{ColorCheckerDetection_1.outputData}",
                "input": "{ColorCheckerDetection_1.input}",
                "correctionMethod": "full",
                "keepImageName": false
            }
        },
        "ColorCheckerDetection_1": {
            "nodeType": "ColorCheckerDetection",
            "position": [
                -108,
                -19
            ],
            "inputs": {
                "input": "{CameraInit_1.output}"
            }
        },
        "CopyFiles_1": {
            "nodeType": "CopyFiles",
            "position": [
                279,
                -10
            ],
            "inputs": {
                "inputFiles": [
                    "{ColorCheckerCorrection_1.output}"
                ]
            }
        }
    }
}