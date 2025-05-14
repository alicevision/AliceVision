{
    "header": {
        "nodesVersions": {
            "CameraInit": "12.0",
            "Publish": "1.3"
        },
        "releaseVersion": "2025.1.0-develop",
        "fileVersion": "2.0",
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
        "Publish_1": {
            "nodeType": "Publish",
            "position": [
                3200,
                0
            ],
            "inputs": {
                "inputFiles": [
                    "{CameraInit_1.output}",
                ]
            }
        }
    }
}