from ..utils import *
from ..constants import *

from aliceVision import sfmData as av

##################
### List of functions:
# - View(imagePath = "", viewId = UndefinedIndexT, intrinsicId = UndefinedIndexT, poseId = UndefinedIndexT, width = 0, height = 0, rigId = UndefinedIndexT, subPoseId = UndefinedIndexT, metadata = map<string, string>()) => DONE
# - View* shallowClone() => DONE
# - View* clone() => DONE
# - operator==(other) => DONE
# - [inline] operator!=(other) => DONE
# - ImageInfo getImage() => DONE
# - shared_ptr<ImageInfo> getImageInfo() => DONE
# - IndexT getViewId() => DONE
# - IndexT getIntrinsicId() => DONE
# - IndexT getPoseId() => DONE
# - IndexT getRigId() => DONE
# - IndexT getSubPoseId() => DONE
# - IndexT getFrameId() => DONE
# - IndexT getResectionId() => DONE
# - bool isPartOfRig() => DONE
# - bool isPoseIndependent() => DONE
# - void setViewId(IndexT) => DONE
# - void setIntrinsicId(IndexT) => DONE
# - void setPoseId(IndexT) => DONE
# - void setIndependantPose(bool) => DONE
# - void setRigAndSubPoseId(IndexT rigId, IndexT subPoseId) => DONE
# - void setFrameId(IndexT) => DONE
# - vector<IndexT> getAncestors() => DONE
# - void addAncestor(IndexT) => DONE
# - void setResectionId(IndexT) => DONE
##################


def test_view_default_constructor():
    """ Test creating a View object with default parameters and accessing its members. """
    view = av.View()

    assert view.getAncestors() == (), "Default View object should have an empty list of ancestors"
    assert view.getFrameId() == 4294967295, "Default frame ID should be 4294967295"
    assert view.getFrameId() == view.getIntrinsicId() == view.getPoseId() == view.getResectionId() == view.getRigId() \
        == view.getSubPoseId() == view.getViewId(), "All IDs should be unitialized"

    ii = view.getImage()
    assert ii, "An ImageInfo object should be available for a default View object"
    assert ii.getImgSize() == (0, 0), "Default View size should be (0, 0)"


def test_view_constructor():
    """ Test creating a View object with values and accessing its members. """
    view = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)

    ii = view.getImage()
    assert ii, "An ImageInfo object should be available"
    assert ii.getImgSize() == (IMAGE_WIDTH, IMAGE_HEIGHT), "Image size does not correspond to the one set in constructor"
    assert ii.getImagePath() == IMAGE_PATH, "Image path does not correspond to the one set in constructor"

    assert view.getViewId() == VIEW_ID, "Unexpected view ID"
    assert view.getIntrinsicId() == INTRINSIC_ID, "Unexpected intrinsic ID"
    assert view.getRigId() == RIG_ID, "Unexpected rig ID"
    assert view.getPoseId() == POSE_ID, "Unexpected pose ID"
    assert view.getSubPoseId() == SUBPOSE_ID, "Unexpected subpose ID"

    assert view.getFrameId() == 4294967295, "Frame ID should be unset"


def test_view_compare():
    """ Test creating two Views objects and comparing them using the '==' and '!=' operators. """
    v1 = av.View()
    v2 = av.View()

    assert v1 == v2, "Two default View objects should be equal"

    # Frame ID is not taken into account when comparing views: the views should be equal even with different frame IDs
    v1.setFrameId(98765)
    assert v1 == v2, "View objects should be equal, even with different frame IDs"

    v1.setViewId(VIEW_ID)
    assert v1 != v2, "View objects should be different since the view IDs differ"


def test_view_deep_copy():
    """ Test creating a View object and its deep copy, and checking whether the copy is indeed deep. """
    v1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    v2 = v1.clone()

    assert v1 == v2, "The two View objects should be equal since they're copies"
    assert v1.getImage() == v2.getImage()

    v1.setViewId(VIEW_ID + 1)
    ii = v1.getImage()
    ii.setWidth(ii.getWidth() + 1)

    assert v1 != v2, "The two View objects should have different view IDs since the copy is deep"
    assert v1.getImage() != v2.getImage(), "The two View objects should have different ImageInfos since the copy is deep"


def test_view_shallow_copy():
    """ Test creating a View object and its shallow copy, and checking whether the copy is indeed shallow. """
    v1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    v2 = v1.shallowClone()

    assert v1 == v2, "The two View objects should be equal since they're copies"
    assert v1.getImage() == v2.getImage()

    v1.setViewId(VIEW_ID + 1)
    ii = v1.getImage()
    ii.setWidth(ii.getWidth() + 1)

    assert v1 != v2, "The two View objects should have different view IDs since the copy is shallow"
    assert v1.getImage() == v2.getImage(), "The two View objects should have identical ImageInfos since the copy is shallow"


def test_view_get_set_values():
    """ Test creating a View object with default parameters and then set and get its values. """
    view = av.View()

    view.setViewId(VIEW_ID)
    assert view.getViewId() == VIEW_ID, "View ID has not been correctly set"

    view.setIntrinsicId(123)
    assert view.getIntrinsicId() == 123, "Intrinsic ID has not been correctly set"

    view.setPoseId(345)
    assert view.getPoseId() == 345, "Pose ID has not been correctly set"

    assert not view.isPartOfRig(), "Rig ID is not defined, the View object cannot be part of a rig"

    view.setRigAndSubPoseId(456, 567)
    assert view.getRigId() == 456, "Rig ID has not been correctly set"
    assert view.getSubPoseId() == 567, "Sub-pose ID has not been correctly set"
    assert view.isPartOfRig(), "Rig ID is set, the View object should be part of a rig"

    view.setResectionId(789)
    assert view.getResectionId() == 789, "Resection ID has not been correctly set"

    view.setFrameId(98765)
    assert view.getFrameId() == 98765, "Frame ID has not been correctly set"

    assert view.isPoseIndependant()
    view.setIndependantPose(False)
    assert not view.isPoseIndependant()


def test_view_get_image():
    """ Test creating a View object with some parameters and ensuring that the ImageInfo object is accurate and correctly updated. """
    view = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)

    ii1 = view.getImageInfo()
    ii2 = view.getImage()

    assert ii1 == ii2, "The ImageInfo object should be the same no matter the way used to retrieve it"
    
    assert ii1.getWidth() == IMAGE_WIDTH and ii1.getHeight() == IMAGE_HEIGHT, "The ImageInfo object size should be the one set when creating the View object"
    assert ii2.getImgSize() == (IMAGE_WIDTH, IMAGE_HEIGHT), "The ImageInfo object size should be the one set when creating the View object"

    ii1.setWidth(IMAGE_WIDTH + 100)
    assert ii2.getWidth() == IMAGE_WIDTH + 100, "The width of the ImageInfo object has been modified, the structure should have been updated"

    ii2.setHeight(IMAGE_HEIGHT + 100)
    assert ii1.getHeight() == IMAGE_HEIGHT + 100, "The height of the ImageInfo object has been modified, the structure should have been updated"


def test_view_get_set_ancestors():
    """ Test creating View objects and adding/retrieving their ancestors. """
    v1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    v2 = av.View()

    assert len(v1.getAncestors()) == 0 and len(v2.getAncestors()) == 0, "No ancestor has been added to any of the View objects"

    v2.addAncestor(VIEW_ID)
    assert len(v2.getAncestors()) == 1, "An ancestor has been added to the default View object"
    assert VIEW_ID in v2.getAncestors(), "The default View object should have {} as its ancestor".format(VIEW_ID)
    assert len(v1.getAncestors()) == 0, "The non-default View object should still have no ancestor"
