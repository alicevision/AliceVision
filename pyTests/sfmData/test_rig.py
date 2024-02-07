"""
Collection of unit tests for the Rig class.
"""

from aliceVision import sfmData as av

##################
### List of functions:
# - struct RigSubPose(geometry::Pose3& pose = geometry::Pose3(),
#                     ERigSubPoseStatus status = ERigSubPoseStatus::UNINITIALIZED)
#                     => DONE !!! Pose3 not binded !!!
# - operator==(RigSubPose& other) => DONE
# - Rig(unsinged int nbSubPoses = 0) => DONE
# - operator==(other) => DONE
# - bool isInitialized() => DONE
# - bool isFullyCalibrated() => DONE
# - size_t getNbSubPoses() => DONE
# - vector<RigSubPose>& getSubPoses() => DONE
# - RigSubPose& getSubPose(IndexT index) => DONE
# - void setSubPose(IndexT index, RigSubPose& rigSubPose) => DONE
# - void reset() => DONE
##################

def test_rig_rigsubpose_constructor():
    """ Test creating a RigSubPose structure and checking whether its values are
    correctly initialized. """
    subpose1 = av.RigSubPose()
    assert subpose1.status == av.ERigSubPoseStatus_UNINITIALIZED

    # TODO: check that subpose1.pose = geometry::Pose3()
    # TODO: create RigSubPose with initial pose and status values


def test_rig_rigsubpose_compare():
    """ Test creating RigSubPose structures and checking their equality with the '==' operator. """
    subpose1 = av.RigSubPose()
    subpose2 = av.RigSubPose()

    assert subpose1 == subpose2

    subpose1.status = av.ERigSubPoseStatus_ESTIMATED
    assert not subpose1 == subpose2


def test_rig_constructor():
    """ Test creating a Rig object and checking whether its values are correctly initialized. """
    rig1 = av.Rig()  # default values, nbSubPoses = 0
    assert len(rig1.getSubPoses()) == 0

    rig2 = av.Rig(5)
    assert len(rig2.getSubPoses()) == 5


def test_rig_compare():
    """ Test creating Rig objects and comparing them with the '==' operator. """
    rig1 = av.Rig()
    rig2 = av.Rig()

    assert rig1 == rig2, \
        "The two Rig objects have no subposes, they should be equal"

    rig2 = av.Rig(5)
    assert not rig1 == rig2


def test_rig_is_initialized():
    """ Test creating a Rig object and checking whether at least one of its subposes
    is initialized. """
    rig = av.Rig(5)
    assert not rig.isInitialized(), \
        "The Rig object has just been created, no subpose has been initialized"

    # Set an uninitialized subpose
    rig.setSubPose(0, av.RigSubPose())
    assert not rig.isInitialized(), \
        "The subpose that has been added was uninitialized"

    subpose = av.RigSubPose()
    subpose.status = av.ERigSubPoseStatus_ESTIMATED
    rig.setSubPose(1, subpose)
    assert rig.isInitialized(), \
        "The subpose that has been added was initialized"

    rig.getSubPose(1).status = av.ERigSubPoseStatus_UNINITIALIZED
    assert not rig.isInitialized(), \
        "The initialized subpose's status has been changed to 'unitialized'"


def test_rig_is_fully_calibrated():
    """ Test creating a Rig object and checking whether all of its subposes are initialized. """
    rig = av.Rig(5)
    assert not rig.isFullyCalibrated()

    for i in range(5):
        subpose = av.RigSubPose()
        subpose.status = av.ERigSubPoseStatus_ESTIMATED
        rig.setSubPose(i, subpose)

        if i < 4:  # Check for every subpose except the last one
            assert not rig.isFullyCalibrated(), "At least one subpose is uninitialized"

    assert rig.isFullyCalibrated(), "All the subposes' status have been changed to 'estimated'"


def test_rig_get_nb_subposes():
    """ Test creating a Rig object and retrieving its number of subposes. """
    rig = av.Rig(5)
    assert len(rig.getSubPoses()) == rig.getNbSubPoses() == 5

    rig = av.Rig(3)
    assert len(rig.getSubPoses()) == rig.getNbSubPoses() == 3

    rig = av.Rig()
    assert len(rig.getSubPoses()) == rig.getNbSubPoses() == 0


def test_rig_get_set_subposes():
    """ Test creating a Rig object and retrieving/modifying its subposes. """
    rig = av.Rig(5)

    for subpose in rig.getSubPoses():
        assert subpose.status == av.ERigSubPoseStatus_UNINITIALIZED

    # Modify the existing subposes
    for i in range(len(rig.getSubPoses())):
        subpose = rig.getSubPose(i)
        if i % 2 == 0:
            subpose.status = av.ERigSubPoseStatus_ESTIMATED
        else:
            subpose.status = av.ERigSubPoseStatus_CONSTANT

    # Check that the subposes have been modified as expected
    assert rig.isFullyCalibrated()

    # Replace the existing subposes with default ones
    for i in range(rig.getNbSubPoses()):
        rig.setSubPose(i, av.RigSubPose())

    # Check that the subposes have been replaced with default ones
    subposes_statuses = [subpose.status == av.ERigSubPoseStatus_UNINITIALIZED
        for subpose in rig.getSubPoses()]
    assert all(subposes_statuses)


def test_rig_reset():
    """ Test creating a Rig object, setting its subposes, then resetting them. """
    rig = av.Rig(5)

    # Set non-default subposes
    for i in range(rig.getNbSubPoses()):
        rig.getSubPose(i).status = av.ERigSubPoseStatus_ESTIMATED

    assert rig.isFullyCalibrated()

    # Reset the subposes
    rig.reset()

    # Check that the subposes have been reset
    subposes_statuses = [subpose.status == av.ERigSubPoseStatus_UNINITIALIZED
        for subpose in rig.getSubPoses()]
    assert all(subposes_statuses)
