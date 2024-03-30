# This file is part of the AliceVision project.
# Copyright (c) 2024 AliceVision contributors.
# This Source Code Form is subject to the terms of the Mozilla Public License,
# v. 2.0. If a copy of the MPL was not distributed with this file,
# You can obtain one at https://mozilla.org/MPL/2.0/.

#Found on https://video.stackexchange.com/questions/23382/maya-python-worldspace-to-screenspace-coordinates

#A snippet to use in maya to check the numerical correctness of the projection

import maya.cmds as cmds
import maya.OpenMaya as om
import sys

def worldSpaceToScreenSpace(camera, worldPoint):

    # get current resolution, modify according to your needs
    resWidth = 1100
    resHeight = 850

    # get the dagPath to the camera shape node to get the world inverse matrix
    selList = om.MSelectionList()
    selList.add(camera)
    dagPath = om.MDagPath()
    selList.getDagPath(0,dagPath)
    dagPath.extendToShape()
    camInvMtx = dagPath.inclusiveMatrix().inverse()

    # use a camera function set to get projection matrix, convert the MFloatMatrix 
    # into a MMatrix for multiplication compatibility
    fnCam = om.MFnCamera(dagPath)
    mFloatMtx = fnCam.projectionMatrix()
    projMtx = om.MMatrix(mFloatMtx.matrix)
        

    # multiply all together and do the normalisation    
    mPoint = om.MPoint(worldPoint[0], worldPoint[1], worldPoint[2]) * camInvMtx * projMtx
    x = (((mPoint[0] / mPoint[3]) / 2.0) + 0.5) * resWidth
    y = (((-mPoint[1] / mPoint[3]) / 2.0) + 0.5) * resHeight
    
    return [x,y]

cameras = (cmds.ls(type="camera"))

for camera in cameras:
    print(camera)
    print(worldSpaceToScreenSpace(camera, [0, 0, 1, 1]))