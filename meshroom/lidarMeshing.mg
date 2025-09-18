{
    "header": {
        "releaseVersion": "2025.1.0",
        "fileVersion": "2.0",
        "nodesVersions": {
            "CopyFiles": "1.3",
            "ImportE57": "1.0",
            "LidarDecimating": "1.0",
            "LidarMerging": "1.0",
            "LidarMeshing": "1.0"
        },
        "template": true
    },
    "graph": {
        "ImportE57_1": {
            "nodeType": "ImportE57",
            "position": [
                0,
                0
            ],
            "inputs": {}
        },
        "LidarDecimating_1": {
            "nodeType": "LidarDecimating",
            "position": [
                400,
                0
            ],
            "inputs": {
                "input": "{LidarMeshing_1.outputJson}"
            }
        },
        "LidarMerging_1": {
            "nodeType": "LidarMerging",
            "position": [
                600,
                0
            ],
            "inputs": {
                "input": "{LidarDecimating_1.outputJson}"
            }
        },
        "LidarMeshing_1": {
            "nodeType": "LidarMeshing",
            "position": [
                200,
                0
            ],
            "inputs": {
                "input": "{ImportE57_1.output}"
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
                    "{LidarMerging_1.output}"
                ]
            }
        }
    }
}