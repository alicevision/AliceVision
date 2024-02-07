"""
Collection of unit tests for the SfMData class.
"""

import os

from aliceVision import sfmData as av
from ..constants import IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, IMAGE_WIDTH, \
    IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA

##################
### List of functions:
# - operator==(other)
# - [inline] operator!=(other)
# - Views& getViews() => DONE
# - ImageInfos& getAncestors()
# - Poses& getPoses() => DONE
# - Rigs& getRigs() => DONE
# - Intrinsics& getIntrinsics() => DONE !!! Intrinsics derived classes not fully binded !!!
# - Landmarks& getLandmarks() => DONE
# - Constraints2D& getConstraints2D() => DONE
# - RotationPriors& getRotationPriors() => DONE
# - vector<string>& getRelativeFeaturesFolders() => DONE
# - vector<string>& getRelativeMatchesFolders() => DONE
# - vector<string> getFeaturesFolders() => DONE
# - vector<string> getMatchesFolders() => DONE
# - set<IndexT> getValidViews()
# - set<IndexT> getReconstructedIntrinsics()
# - IntrinsicBase* getIntrinsicPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(View& v)
# - set<IndexT> getViewsKeys() => DONE
# - bool isPoseAndIntrinsicDefined(View* view)
# - bool isPoseAndIntrinsicDefined(IndexT viewId)
# - bool existsPose(View& view)
# - View& getView(IndexT viewId) => DONE
# - View* getViewPtr(IndexT viewId) => DONE
# - shared_ptr<View> getViewSharedPtr(IndexT viewId) => DONE
# - CameraPose getPose(View& view)
# - CameraPose& getAbsolutePose(IndexT poseId)
# - Rig& getRig(View& view)
# - set<EImageDescriberType> getLandmarkDescTypes() !!! EImageDescriberType not correctly binded !!!
# - map<EImageDescriberType, int> getLandmarkDescTypesUsages() !!! EImageDescriberType not correctly binded !!!
# - ExposureSetting getMedianCameraExposureSetting()
# - [inline] void addFeaturesFolder(string& folder) => DONE
# - void addFeaturesFolders(vector<string>& folders) => DONE
# - [inline] void addMatchesFolder(string& folder) => DONE
# - void addMatchesFolders(vector<string>& folders) => DONE
# - [inline] void setFeaturesFolders(vector<string>& folders) => DONE
# - [inline] void setMatchesFolders(vector<string>& folders) => DONE
# - void setAbsolutePath(string& path) => DONE
# - void setPose(View& view, CameraPose& pose)
# - void setAbsolutePose(IndexT poseId, CameraPose& pose)
# - void erasePose(IndexT poseId, bool noThrow = false)
# - void resetRigs()
# - void addAncestor(IndexT ancestorId, shared_ptr<ImageInfo> image)
# - void combine(SfMData& sfmData) => DONE
# - void clear() => DONE
# - void resetParameterStates()
##################

def test_sfmdata_get_views():
    """ Test creating an emtpy SfMData object, retrieving and editing its Views. """
    data = av.SfMData()
    views = data.getViews()
    assert len(views) == 0, "The SfMData object is emtpy, there should not be any View in it"

    # Create a View object and add it to the list
    view = av.View()
    views[VIEW_ID] = view
    assert len(data.getViews()) == len(views) == 1, "The list of Views should have been updated"


def test_sfmdata_get_poses():
    """ Test creating an empty SfMData object, retrieving and editing its Poses. """
    data = av.SfMData()
    poses = data.getPoses()
    assert len(poses) == 0, "The SfMData object is empty, there should not be any Pose in it"

    # Create a Pose object and add it to the list
    camera_pose = av.CameraPose()
    poses[POSE_ID] = camera_pose
    assert len(data.getPoses()) == len(poses) == 1, "The list of Poses should have been updated"


def test_sfmdata_get_rigs():
    """ Test creating an empty SfMData object, retrieving and editing its Rigs. """
    data = av.SfMData()
    rigs = data.getRigs()
    assert len(rigs) == 0, "The SfMData object is empty, there should not be any Rig in it"

    # Create a Rig object and add it to the list
    rig = av.Rig(5)
    rigs[RIG_ID] = rig
    assert len(data.getRigs()) == len(rigs) == 1, "The list of Rigs should have been updated"


def test_sfmdata_get_intrinsics():
    """ Test creating an empty SfMData object, retrieving and editing its Intrinsics. """
    data = av.SfMData()
    intrinsics = data.getIntrinsics()
    assert len(intrinsics) == 0, \
        "The SfMData object is empty, there should not be any Intrinsic in it"

    # TODO: Add Intrinsic object to the list once derived classes are fully binded
    # Create an Intrinsic object and add it to the list
    # intrinsic = av.IntrinsicBase()
    # intrinsics[INTRINSIC_ID] = intrinsic
    # assert len(data.getIntrinsics()) == len(intrinsics) == 1, \
    #     "The list of Intrinsics should have been updated"


def test_sfmdata_get_landmarks():
    """ Test creating an empty SfMData object, retrieving and editing its Landmarks. """
    data = av.SfMData()
    landmarks = data.getLandmarks()
    assert len(landmarks) == 0, \
        "The SfMData object is empty, there should not be any Landmark in it"

    # Create a Landmark object and add it to the list
    landmark = av.Landmark()
    landmarks[123] = landmark
    assert len(data.getLandmarks()) == len(landmarks) == 1, \
        "The list of Landmarks should have been updated"


def test_sfmdata_get_constraints2d():
    """" Test creating an empty SfMData object, retrieving and editing its Constraints2D. """
    data = av.SfMData()
    constraints = data.getConstraints2D()
    assert len(constraints) == 0, \
        "The SfMData object is empty, there should not be any Constraint2D in it"

    # Create a Constraint2D object and add it to the list
    constraint = av.Constraint2D()
    constraints.append(constraint)
    assert len(data.getConstraints2D()) == len(constraints) == 1, \
        "The list of Constraints2D should have been updated"


def test_sfmdata_get_rotationpriors():
    """" Test creating an empty SfMData object, retrieving and editing its RotationPriors. """
    data = av.SfMData()
    priors = data.getRotationPriors()
    assert len(priors) == 0, \
        "The SfMData object is empty, there should not be any RotationPrior in it"

    # Create a Constraint2D object and add it to the list
    prior = av.RotationPrior()
    priors.append(prior)
    assert len(data.getRotationPriors()) == len(priors) == 1, \
        "The list of RotationPriors should have been updated"


def test_sfmdata_get_views_keys():
    """ Test creating an empty SfMData object, adding a few Views in it and retrieving the
    keys for its views. """
    data = av.SfMData()
    views = data.getViews()
    assert len(views) == 0, "The SfMData object is empty, there should not be any View in it"

    # Create View objects
    view_ids = [12, 23, 34, 45, 56]
    for view_id in view_ids:
        view = av.View()
        view.setViewId(view_id)
        views[view_id] = view

    assert len(data.getViews()) == len(view_ids), "The SfMData object should not be empty anymore"

    # Check the keys
    keys = data.getViewsKeys()
    assert len(keys) == len(view_ids), "There should be as many View objects as view IDs"
    key_checks = [view_id in keys for view_id in view_ids]
    assert all(key_checks), \
        "All the view IDs should be in the list of View keys"


def test_sfmdata_get_view():
    """ Test creating an empty SfMData object, filling it up with Views, and then
    getting specific Views. """
    data = av.SfMData()
    views = data.getViews()

    for i in range(5):
        view = av.View(IMAGE_PATH, VIEW_ID + i, INTRINSIC_ID + i, POSE_ID + i, IMAGE_WIDTH + i,
        IMAGE_HEIGHT + i, RIG_ID + i, SUBPOSE_ID + i, METADATA)
        views[VIEW_ID + i] = view

    assert len(data.getViews()) == len(views) == 5

    # Regular getter
    for i in range(5):
        view = data.getView(VIEW_ID + i)
        assert view.getIntrinsicId() == INTRINSIC_ID + i
        assert view.getRigId() == RIG_ID + i
        assert view.getPoseId() == POSE_ID + i
        assert view.getSubPoseId() == SUBPOSE_ID + i
        assert view.getImageInfo().getImgSize() == (IMAGE_WIDTH + i, IMAGE_HEIGHT + i)
        assert view.getImageInfo().getImagePath() == IMAGE_PATH

    # Pointer getter
    for i in range(5):
        view = data.getViewPtr(VIEW_ID + i)
        assert view.getIntrinsicId() == INTRINSIC_ID + i
        assert view.getRigId() == RIG_ID + i
        assert view.getPoseId() == POSE_ID + i
        assert view.getSubPoseId() == SUBPOSE_ID + i
        assert view.getImageInfo().getImgSize() == (IMAGE_WIDTH + i, IMAGE_HEIGHT + i)
        assert view.getImageInfo().getImagePath() == IMAGE_PATH

    # Shared pointer getter
    for i in range(5):
        view = data.getViewSharedPtr(VIEW_ID + i)
        assert view.getIntrinsicId() == INTRINSIC_ID + i
        assert view.getRigId() == RIG_ID + i
        assert view.getPoseId() == POSE_ID + i
        assert view.getSubPoseId() == SUBPOSE_ID + i
        assert view.getImageInfo().getImgSize() == (IMAGE_WIDTH + i, IMAGE_HEIGHT + i)
        assert view.getImageInfo().getImagePath() == IMAGE_PATH


def test_sfmdata_combine():
    """ Test creating two SfMData objects before combining them into a single one. """
    data1 = av.SfMData()
    views1 = data1.getViews()

    # Create View objects for first SfMData
    view_ids1 = [12, 23, 34, 45, 56]
    for view_id in view_ids1:
        view = av.View()
        view.setViewId(view_id)
        views1[view_id] = view

    assert len(data1.getViews()) == len(view_ids1), \
        "The SfMData object should contain several View objects"

    data2 = av.SfMData()
    views2 = data2.getViews()

    # Create View objects for second SfMData
    view_ids2 = [67, 78, 89, 90]
    for view_id in view_ids2:
        view = av.View()
        view.setViewId(view_id)
        views2[view_id] = view

    assert len(data2.getViews()) == len(view_ids2), \
        "The SfMData object should contain several View objects"

    # Combine the two SfMData objects
    data1.combine(data2)
    assert len(data1.getViews()) == len(view_ids1) + len(view_ids2), \
        "The first SfMData object should now contain the sum of the Views"

    keys = data1.getViewsKeys()
    key_checks = [view_id in keys for view_id in view_ids1 + view_ids2]
    assert all(key_checks), \
        "All the view IDs should be in the View keys from the combined SfMData object"

    # Combining several SfMData objects from one default one
    data3 = av.SfMData()
    assert len(data3.getViews()) == 0, "The SfMData object should be empty"

    # Reset first SfMData object to its initial list of Views
    for view_id in view_ids2:
        del views1[view_id]
    assert len(data1.getViews()) == len(view_ids1), \
        "The first SfMData object should have been reset to its original list of Views"

    data3.combine(data1)
    data3.combine(data2)
    assert len(data3.getViews()) == len(view_ids1) + len(view_ids2), \
        "The two SfMData objects should have been combined in the third one"
    keys = data3.getViewsKeys()
    key_checks = [view_id in keys for view_id in view_ids1 + view_ids2]
    assert all(key_checks), \
        "All the view IDs should be in the View keys from the combined SfMData object"


def test_sfmdata_clear():
    """ Test creating manually an SfMData object before clearing its content. """
    data = av.SfMData()
    views = data.getViews()

    # Create View objects
    view_ids = [12, 23, 34, 45, 56]
    for view_id in view_ids:
        view = av.View()
        view.setViewId(view_id)
        views[view_id] = view

    assert len(data.getViewsKeys()) > 0

    data.clear()
    keys_cleared = data.getViewsKeys()
    assert len(keys_cleared) == 0


def test_sfmdata_get_set_folders():
    """ Test setting and retrieving features and matches folders for an SfMData object. """
    data = av.SfMData()

    # Add relative features and matches folders with duplicates
    relative_folder = ".."
    data.addFeaturesFolders([relative_folder, relative_folder])
    data.addMatchesFolders([relative_folder, relative_folder])

    # Check that the folders were correctly added and the duplicates removed
    assert len(data.getFeaturesFolders()) == 1, \
        "Two identical features folders have been added, only one should have been kept"
    assert data.getFeaturesFolders()[0] == relative_folder
    assert len(data.getMatchesFolders()) == 1, "\
        Two identical matches folders have been added, only one should have been kept"
    assert data.getMatchesFolders()[0] == relative_folder

    # Set absolute path
    filename = "internal_folders.sfm"
    abs_filename = os.path.abspath(os.path.dirname(__file__)) + "/" + filename
    data.setAbsolutePath(abs_filename)

    # Check that the folders were kept and are now absolute paths
    assert len(data.getFeaturesFolders()) == 1, \
        "The previously added features folder should have remained in the list"
    assert os.path.isabs(data.getFeaturesFolders()[0]), \
        "The absolute path has been set: the features folder's path should be absolute as well"
    assert os.path.relpath(data.getFeaturesFolders()[0]) == relative_folder, \
        "The absolute path for the features folder should correspond to the provided relative one"
    assert len(data.getMatchesFolders()) == 1, \
        "The previously added matches folder should have remained in the list"
    assert os.path.isabs(data.getMatchesFolders()[0]), \
        "The absolute path has been set: the matches folder's path should be absolute as well"
    assert os.path.relpath(data.getMatchesFolders()[0]) == relative_folder, \
        "The absolute path for the matches folder should correspond to the provided relative one"

    # Check that relative folders are still valid
    assert len(data.getRelativeFeaturesFolders()) == 1 and \
        data.getRelativeFeaturesFolders()[0] == relative_folder
    assert len(data.getRelativeMatchesFolders()) == 1 and \
        data.getRelativeMatchesFolders()[0] == relative_folder

    # Add single features and matches folders
    other_folder = "../other"
    data.addFeaturesFolder(other_folder)
    data.addMatchesFolder(other_folder)
    assert len(data.getFeaturesFolders()) == 2, \
        "A second features folder should have been added to the list"
    assert len(data.getMatchesFolders()) == 2, \
        "A second matches folder should have been addded to the list"

    assert os.path.relpath(data.getFeaturesFolders()[0]) == relative_folder
    assert os.path.relpath(data.getFeaturesFolders()[1]) == other_folder
    assert os.path.relpath(data.getMatchesFolders()[0]) == relative_folder
    assert os.path.relpath(data.getMatchesFolders()[1]) == other_folder

    # Reset features and matches folders and add new ones
    new_folder1 = "../new"
    new_folder2 = "../.."
    new_folder3 = "folder"
    new_folders = [new_folder1, new_folder2, new_folder3]

    data.setFeaturesFolders(new_folders)
    data.setMatchesFolders(new_folders)

    assert len(data.getFeaturesFolders()) == 3
    assert len(data.getMatchesFolders()) == 3
    for i in range(len(data.getFeaturesFolders())):
        assert os.path.relpath(data.getFeaturesFolders()[i]) == new_folders[i]
        assert os.path.relpath(data.getMatchesFolders()[i]) == new_folders[i]
