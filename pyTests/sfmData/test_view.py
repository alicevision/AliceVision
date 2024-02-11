"""
Collection of unit tests for the View class.
"""

from aliceVision import sfmData as av
from ..constants import IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, \
    IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA

##################
### List of functions:
# - View(imagePath = "", viewId = UndefinedIndexT, intrinsicId = UndefinedIndexT,
#        poseId = UndefinedIndexT, width = 0, height = 0, rigId = UndefinedIndexT,
#        subPoseId = UndefinedIndexT, metadata = map<string, string>()) => DONE
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
    assert view.getFrameId() == view.getIntrinsicId() == view.getPoseId() \
        == view.getResectionId() == view.getRigId() \
        == view.getSubPoseId() == view.getViewId(), "All IDs should be unitialized"

    image_info = view.getImage()
    assert image_info, "An ImageInfo object should be available for a default View object"
    assert image_info.getImgSize() == (0, 0), "Default View size should be (0, 0)"


def test_view_constructor():
    """ Test creating a View object with values and accessing its members. """
    view = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, IMAGE_HEIGHT,
        RIG_ID, SUBPOSE_ID, METADATA)

    image_info = view.getImage()
    assert image_info, "An ImageInfo object should be available"
    assert image_info.getImgSize() == (IMAGE_WIDTH, IMAGE_HEIGHT), \
        "Image size does not correspond to the one set in constructor"
    assert image_info.getImagePath() == IMAGE_PATH, \
        "Image path does not correspond to the one set in constructor"

    assert view.getViewId() == VIEW_ID, "Unexpected view ID"
    assert view.getIntrinsicId() == INTRINSIC_ID, "Unexpected intrinsic ID"
    assert view.getRigId() == RIG_ID, "Unexpected rig ID"
    assert view.getPoseId() == POSE_ID, "Unexpected pose ID"
    assert view.getSubPoseId() == SUBPOSE_ID, "Unexpected subpose ID"

    assert view.getFrameId() == 4294967295, "Frame ID should be unset"


def test_view_compare():
    """ Test creating two Views objects and comparing them using the '==' and '!=' operators. """
    view1 = av.View()
    view2 = av.View()

    assert view1 == view2, "Two default View objects should be equal"

    # Frame ID is not taken into account when comparing views:
    # the views should be equal even with different frame IDs
    view1.setFrameId(98765)
    assert view1 == view2, "View objects should be equal, even with different frame IDs"

    view1.setViewId(VIEW_ID)
    assert view1 != view2, "View objects should be different since the view IDs differ"


def test_view_deep_copy():
    """ Test creating a View object and its deep copy, and checking whether
        the copy is indeed deep. """
    view1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH,
        IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    view2 = view1.clone()

    assert view1 == view2, "The two View objects should be equal since they're copies"
    assert view1.getImage() == view2.getImage()

    view1.setViewId(VIEW_ID + 1)
    image_info = view1.getImage()
    image_info.setWidth(image_info.getWidth() + 1)

    assert view1 != view2, \
        "The two View objects should have different view IDs since the copy is deep"
    assert view1.getImage() != view2.getImage(), \
        "The two View objects should have different ImageInfos since the copy is deep"


def test_view_shallow_copy():
    """ Test creating a View object and its shallow copy, and checking whether
    the copy is indeed shallow. """
    view1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH,
        IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    view2 = view1.shallowClone()

    assert view1 == view2, "The two View objects should be equal since they're copies"
    assert view1.getImage() == view2.getImage()

    view1.setViewId(VIEW_ID + 1)
    image_info = view1.getImage()
    image_info.setWidth(image_info.getWidth() + 1)

    assert view1 != view2, \
        "The two View objects should have different view IDs since the copy is shallow"
    assert view1.getImage() == view2.getImage(), \
        "The two View objects should have identical ImageInfos since the copy is shallow"


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
    """ Test creating a View object with some parameters and ensuring that the ImageInfo
    object is accurate and correctly updated. """
    view = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH,
        IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)

    image_info1 = view.getImageInfo()
    image_info2 = view.getImage()

    assert image_info1 == image_info2, "The ImageInfo object should be the same no matter \
        the way used to retrieve it"

    assert image_info1.getWidth() == IMAGE_WIDTH and image_info1.getHeight() == IMAGE_HEIGHT, \
        "The ImageInfo object size should be the one set when creating the View object"
    assert image_info2.getImgSize() == (IMAGE_WIDTH, IMAGE_HEIGHT), \
        "The ImageInfo object size should be the one set when creating the View object"

    image_info1.setWidth(IMAGE_WIDTH + 100)
    assert image_info2.getWidth() == IMAGE_WIDTH + 100, \
        "The width of the ImageInfo object has been modified, the structure \
        should have been updated"

    image_info2.setHeight(IMAGE_HEIGHT + 100)
    assert image_info1.getHeight() == IMAGE_HEIGHT + 100, \
        "The height of the ImageInfo object has been modified, the structure \
            should have been updated"


def test_view_get_set_ancestors():
    """ Test creating View objects and adding/retrieving their ancestors. """
    view1 = av.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH,
        IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    view2 = av.View()

    assert len(view1.getAncestors()) == 0 and len(view2.getAncestors()) == 0, \
        "No ancestor has been added to any of the View objects"

    view2.addAncestor(VIEW_ID)
    assert len(view2.getAncestors()) == 1, \
        "An ancestor has been added to the default View object"
    assert VIEW_ID in view2.getAncestors(), \
        f"The default View object should have {VIEW_ID} as its ancestor"
    assert len(view1.getAncestors()) == 0, \
        "The non-default View object should still have no ancestor"
