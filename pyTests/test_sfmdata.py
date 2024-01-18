from utils import *
import os

import aliceVision as av

##################
### List of functions:
# - operator==(other)
# - [inline] operator!=(other)
# - Views& getViews()
# - ImageInfos& getAncestors()
# - Poses& getPoses()
# - Rigs& getRigs()
# - Intrinsics& getIntrinsics()
# - Landmarks& getLandmarks()
# - Constraints2D& getConstraints2D()
# - RotationPriors& getRotationPriors()
# - vector<string>& getRelativeFeaturesFolders()
# - vector<string>& getRelativeMatchesFolders()
# - vector<string> getFeaturesFolders()
# - vector<string> getMatchesFolders()
# - set<IndexT> getValidViews()
# - set<IndexT> getReconstructedIntrinsics()
# - IntrinsicBase* getIntrinsicPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(View& v)
# - set<IndexT> getViewsKeys()
# - bool isPoseAndIntrinsicDefined(View* view)
# - bool isPoseAndIntrinsicDefined(IndexT viewId)
# - bool existsPose(View& view)
# - View& getView(IndexT viewId)
# - View* getViewPtr(IndexT viewId)
# - shared_ptr<View> getViewSharedPtr(IndexT viewId)
# - CameraPose getPose(View& view)
# - CameraPose& getAbsolutePose(IndexT poseId)
# - Rig& getRig(View& view)
# - set<EImageDescriberType> getLandmarkDescTypes() !!! not correctly binded
# - map<EImageDescriberType, int> getLandmarkDescTypesUsages() !!! not correctly binded
# - ExposureSetting getMedianCameraExposureSetting()
# - [inline] void addFeaturesFolder(string& folder)
# - void addFeaturesFolder(string& folder)
# - void addFeaturesFolders(vector<string>& folders)
# - [inline] void addMatchesFolder(string& folder)
# - void addMatchesFolders(vector<string>& folders)
# - [inline] void setFeaturesFolders(vector<string>& folders)
# - [inline] void setMatchesFolders(vector<string>& folders)
# - void setAbsolutePath(string& path)
# - void setPose(View& view, CameraPose& pose)
# - void setAbsolutePose(IndexT poseId, CameraPose& pose)
# - void erasePose(IndexT poseId, bool noThrow = false)
# - void resetRigs()
# - void addAncestor(IndexT ancestorId, shared_ptr<ImageInfo> image)
# - void combine(SfMData& sfmData)
# - void clear()
# - void resetParameterStates()
##################

PATH = os.path.abspath(os.path.dirname(__file__)) + "/data/small.sfm"

def test_sfmdata_load():
    """ Test loading a SfMData from disk. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, PATH, av.sfmDataIO.ALL)
    assert ret == True


def test_sfmdata_get_views_keys():
    """ Test loading a SfMData from disk and retrieving the keys for its views. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, PATH, av.sfmDataIO.ALL)
    keys = data.getViewsKeys()
    print(keys)
    assert len(keys) == 30


def test_sfmdata_clear():
    """ Test loading a SfMData from disk and clearing its content. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, PATH, av.sfmDataIO.ALL)
    keys = data.getViewsKeys()
    assert len(keys) == 30
    data.clear()
    keys_cleared = data.getViewsKeys()
    assert len(keys_cleared) == 0



if __name__ == "__main__":
    print_cyan("\n=== Loading SfMData file... ===")
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, PATH, av.sfmDataIO.ALL)

    if ret:
        print_green("Successfully loaded SfMData file")
    else:
        print_red("Failed to load SfMData file")
        exit(1)
    
    print_cyan("\n=== Retrieving the keys for the views and checking their length... ===")
    keys = data.getViewsKeys()
    if len(keys) == 30:
        print_green("Correct number of keys")
    else:
        print_red("Incorrect number of keys (" + str(len(keys)) + ")")
        print(keys)
        exit(1)
    