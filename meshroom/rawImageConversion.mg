{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CameraInit": "12.0",
            "ImageProcessing": "3.3",
            "Publish": "1.3"
        },
        "template": true
    },
    "graph": {
        "CameraInit_1": {
            "nodeType": "CameraInit",
            "position": [
                -297,
                -56
            ],
            "inputs": {}
        },
        "ImageProcessing_1": {
            "nodeType": "ImageProcessing",
            "position": [
                -58,
                -62
            ],
            "inputs": {
                "input": "{CameraInit_1.output}",
                "extension": "exr",
                "keepImageFilename": true
            }
        },
        "Publish_1": {
            "nodeType": "Publish",
            "position": [
                175,
                -43
            ],
            "inputs": {
                "inputFiles": [
                    "{ImageProcessing_1.output}"
                ]
            }
        }
    }
}