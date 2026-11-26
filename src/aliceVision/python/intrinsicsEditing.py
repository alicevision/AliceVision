"""Hand editing of the parameters of one intrinsic of an SfMData.

Logic of the IntrinsicsEditing Meshroom node, kept outside the node descriptor so that it can be
tested without Meshroom.

The node shows editable fields filled from the input intrinsic, and keeps a snapshot of the values
it filled them with. A field is an edit if and only if it differs from its snapshot value: fields the
user did not touch keep the input's value, even if the input changed since the fields were filled.

Field names are flat ("centerOffsetX", ...). Lock fields take the values "keep", "locked" or "unlocked".
Distortion and undistortion are never read nor modified.
"""

import logging
import math

import numpy as np

from pyalicevision import camera, sfmData, sfmDataIO


# AUTO and the field names are duplicated in the IntrinsicsEditing node
# (meshroom/aliceVision/IntrinsicsEditing.py), which cannot import this module at file level: keep them in sync.
AUTO = "auto"

# Largest intrinsic ID (IndexT is uint32)
MAX_INDEX = 0xFFFFFFFF

NUMERIC_FIELDS = (
    "focalLength",
    "pixelRatio",
    "centerOffsetX",
    "centerOffsetY",
    "sensorWidth",
    "sensorHeight",
    "circleCenterX",
    "circleCenterY",
    "circleRadius",
)

EQUIDISTANT_FIELDS = ("circleCenterX", "circleCenterY", "circleRadius")

POSITIVE_FIELDS = ("focalLength", "pixelRatio", "sensorWidth", "sensorHeight", "circleRadius")

LOCK_FIELDS = ("locked", "scaleLocked", "offsetLocked", "ratioLocked")

LOCK_KEEP = "keep"
LOCK_LOCKED = "locked"
LOCK_UNLOCKED = "unlocked"


def isClose(a, b):
    """Whether two field values are equal, up to the precision lost in the node's attribute round trip."""
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)


def _validFields(typeStr):
    """Numeric fields that make sense for the camera type."""
    if typeStr == "equidistant":
        return NUMERIC_FIELDS
    return tuple(name for name in NUMERIC_FIELDS if name not in EQUIDISTANT_FIELDS)


def _toIntrinsicScaleOffset(intrinsic):
    scaleOffset = camera.IntrinsicScaleOffset.cast(intrinsic)
    if scaleOffset is None:
        raise ValueError(f"Unsupported camera type: {intrinsic.getTypeStr()}")
    return scaleOffset


def _readValues(intrinsic):
    """Values of the numeric fields valid for the intrinsic's camera type."""
    scaleOffset = _toIntrinsicScaleOffset(intrinsic)
    offset = scaleOffset.getOffset().ravel()
    values = {
        "focalLength": float(scaleOffset.getFocalLength()),
        "pixelRatio": float(scaleOffset.getPixelAspectRatio()),
        "centerOffsetX": float(offset[0]),
        "centerOffsetY": float(offset[1]),
        "sensorWidth": float(intrinsic.sensorWidth()),
        "sensorHeight": float(intrinsic.sensorHeight()),
    }
    # A scale with a zero component makes the pixel ratio (fy / fx) and so the focal length NaN, infinite or 0:
    # read them as an unknown focal length with square pixels
    if not (math.isfinite(values["pixelRatio"]) and values["pixelRatio"] > 0.0):
        values["focalLength"] = 0.0
        values["pixelRatio"] = 1.0
    if intrinsic.getTypeStr() == "equidistant":
        equidistant = camera.Equidistant.cast(intrinsic)
        values["circleCenterX"] = float(equidistant.getCircleCenterX())
        values["circleCenterY"] = float(equidistant.getCircleCenterY())
        values["circleRadius"] = float(equidistant.getCircleRadius())
    return values


def _lockStates(intrinsic):
    scaleOffset = _toIntrinsicScaleOffset(intrinsic)
    states = {
        "locked": intrinsic.isLocked(),
        "scaleLocked": scaleOffset.isScaleLocked(),
        "offsetLocked": scaleOffset.isOffsetLocked(),
        "ratioLocked": scaleOffset.isRatioLocked(),
    }
    return {name: LOCK_LOCKED if state else LOCK_UNLOCKED for name, state in states.items()}


def resolveIntrinsicId(data, intrinsicId):
    """Intrinsic ID targeted by the string `intrinsicId` in `data`.

    "auto" stands for the only intrinsic of `data`; any other string is parsed as an integer ID.

    Returns:
        int: The intrinsic ID, or None if "auto" is used without exactly one intrinsic,
        if the string is not an integer, or if the ID is not in `data`.
    """
    intrinsics = data.getIntrinsics()
    if intrinsicId == AUTO:
        if len(intrinsics) != 1:
            return None
        return next(iter(intrinsics.keys()))
    try:
        value = int(intrinsicId)
    except (TypeError, ValueError):
        return None
    # IDs are uint32: an out-of-range key makes the SWIG map lookup raise OverflowError
    if not 0 <= value <= MAX_INDEX:
        return None
    return value if value in intrinsics else None


def readIntrinsic(data, intrinsicId):
    """Camera type and field values of the intrinsic selected by the string `intrinsicId`.

    Returns:
        tuple: (camera type string, {field name: value}) or None if the intrinsic cannot be resolved.
    """
    resolved = resolveIntrinsicId(data, intrinsicId)
    if resolved is None:
        return None
    intrinsic = data.getIntrinsics()[resolved]
    return intrinsic.getTypeStr(), _readValues(intrinsic)


def findEdits(cameraType, fields, snapshot, locks):
    """Fields the user changed.

    A numeric field valid for `cameraType` is an edit if it differs from its snapshot value;
    a lock is an edit if it is not "keep".

    Returns:
        dict: {field name: new value}.
    """
    edits = {}
    for name in _validFields(cameraType):
        if not isClose(fields[name], snapshot[name]):
            edits[name] = float(fields[name])
    for name in LOCK_FIELDS:
        value = locks.get(name, LOCK_KEEP)
        if value != LOCK_KEEP:
            edits[name] = value
    return edits


def checkEdits(edits, defaults):
    """Raise ValueError if an edit would write an invalid value.

    An edited field holding its default value was reset to default or never filled: it is rejected
    rather than written (e.g. a focal length of 0).
    """
    for name, value in edits.items():
        if name not in NUMERIC_FIELDS:
            continue
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number, got {value}")
        if isClose(value, defaults[name]):
            raise ValueError(f"{name} is at its default value {value}: re-select the intrinsic ID to refill it")
        if name in POSITIVE_FIELDS and value <= 0.0:
            raise ValueError(f"{name} must be > 0, got {value}")


def applyEdits(intrinsic, edits, updateInitialFocal):
    """Apply `edits` to `intrinsic`.

    The sensor width is applied before the focal length so that a sensor-only change keeps the focal
    length in mm. The sensor height does not change the focal length in pixels. If the input focal length
    is unknown (<= 0), it stays unknown unless the focal length is edited.

    The initial focal length is the prior that bounds the focal length in bundle adjustment. If
    `updateInitialFocal` is set, it is set to the focal length whenever the focal length in pixels is
    recomputed (focal length, pixel ratio or sensor width edit); otherwise it is set to -1 (no prior).

    Returns:
        list: One "<name>: <old> -> <new>" line per applied edit.
    """
    scaleOffset = _toIntrinsicScaleOffset(intrinsic)
    oldValues = _readValues(intrinsic)
    oldLocks = _lockStates(intrinsic)
    oldScale = np.array(scaleOffset.getScale(), dtype=float).ravel()
    oldInitialScale = np.array(scaleOffset.getInitialScale(), dtype=float).ravel()

    if "sensorWidth" in edits:
        intrinsic.setSensorWidth(edits["sensorWidth"])
    if "sensorHeight" in edits:
        intrinsic.setSensorHeight(edits["sensorHeight"])

    # Decided from the edits, not by comparing the scales: scaling the focal length and the sensor width by the
    # same ratio keeps the scale but is still a focal length edit.
    scaleEdited = "focalLength" in edits or (
        oldValues["focalLength"] > 0.0 and any(name in edits for name in ("pixelRatio", "sensorWidth")))
    if scaleEdited:
        focalLength = edits.get("focalLength", oldValues["focalLength"])
        pixelRatio = edits.get("pixelRatio", oldValues["pixelRatio"])
        scaleOffset.setFocalLength(focalLength, pixelRatio)

    newScale = np.array(scaleOffset.getScale(), dtype=float).ravel()
    if not updateInitialFocal:
        scaleOffset.setInitialScale(np.array([-1.0, -1.0]))
    elif scaleEdited:
        scaleOffset.setInitialScale(newScale if np.all(newScale > 0.0) else np.array([-1.0, -1.0]))
    newInitialScale = np.array(scaleOffset.getInitialScale(), dtype=float).ravel()

    if "centerOffsetX" in edits or "centerOffsetY" in edits:
        scaleOffset.setOffset(np.array([edits.get("centerOffsetX", oldValues["centerOffsetX"]),
                                        edits.get("centerOffsetY", oldValues["centerOffsetY"])]))

    if any(name in edits for name in EQUIDISTANT_FIELDS):
        equidistant = camera.Equidistant.cast(intrinsic)
        if equidistant is None:
            raise ValueError(f"Circle parameters only apply to equidistant cameras, not {intrinsic.getTypeStr()}")
        if "circleCenterX" in edits:
            equidistant.setCircleCenterX(edits["circleCenterX"])
        if "circleCenterY" in edits:
            equidistant.setCircleCenterY(edits["circleCenterY"])
        if "circleRadius" in edits:
            equidistant.setCircleRadius(edits["circleRadius"])

    lockSetters = {
        "locked": lambda state: intrinsic.lock() if state else intrinsic.unlock(),
        "scaleLocked": scaleOffset.setScaleLocked,
        "offsetLocked": scaleOffset.setOffsetLocked,
        "ratioLocked": scaleOffset.setRatioLocked,
    }
    for name in LOCK_FIELDS:
        if name in edits:
            lockSetters[name](edits[name] == LOCK_LOCKED)

    lines = []
    for name in NUMERIC_FIELDS:
        if name in edits:
            note = " (not applied: focal length unknown)" if name == "pixelRatio" and not scaleEdited else ""
            lines.append(f"{name}: {oldValues.get(name)} -> {edits[name]}{note}")
    if not np.array_equal(newScale, oldScale):
        lines.append(f"scale (px): {oldScale.tolist()} -> {newScale.tolist()}")
    if not np.array_equal(newInitialScale, oldInitialScale):
        lines.append(f"initialScale (px): {oldInitialScale.tolist()} -> {newInitialScale.tolist()}")
    for name in LOCK_FIELDS:
        if name in edits:
            lines.append(f"{name}: {oldLocks[name]} -> {edits[name]}")
    return lines


def editSfmData(inputPath, outputPath, intrinsicId, cameraType, fields, snapshot, locks,
                updateInitialFocal, defaults):
    """Load `inputPath`, apply the user's edits to one intrinsic and save the result to `outputPath`.

    Args:
        intrinsicId (str): "auto" or the decimal intrinsic ID.
        cameraType (str): Camera type the user expects; must match the input intrinsic's type.
        fields (dict): Current field values.
        snapshot (dict): Values the fields were filled with.
        locks (dict): Lock fields ("keep", "locked" or "unlocked").
        updateInitialFocal (bool): Use the focal length as prior: set the initial focal length to the
            focal length when the focal length in pixels changes. If False, the initial focal length is set to -1.
        defaults (dict): Default value of each numeric field.

    Raises:
        RuntimeError: If the input cannot be loaded or the output cannot be saved.
        ValueError: If the intrinsic cannot be resolved, the camera type differs, or an edit is invalid.
    """
    data = sfmData.SfMData()
    try:
        loaded = sfmDataIO.load(data, inputPath, sfmDataIO.ALL)
    except Exception as e:
        raise RuntimeError(f"Cannot load SfMData {inputPath}: {e}") from e
    if not loaded:
        raise RuntimeError(f"Cannot load SfMData {inputPath}")

    resolved = resolveIntrinsicId(data, intrinsicId)
    if resolved is None:
        count = len(data.getIntrinsics())
        if intrinsicId == AUTO:
            raise ValueError(f"Intrinsic ID 'auto' requires exactly one intrinsic in the input, found {count}")
        raise ValueError(f"Intrinsic ID '{intrinsicId}' is not an intrinsic of the input")

    intrinsic = data.getIntrinsics()[resolved]
    typeStr = intrinsic.getTypeStr()
    if typeStr != cameraType:
        raise ValueError(f"Camera type mismatch: node expects {cameraType}, input intrinsic {resolved} is {typeStr}")

    edits = findEdits(cameraType, fields, snapshot, locks)
    checkEdits(edits, defaults)

    logging.info(f"Editing intrinsic {resolved} ({typeStr})")
    lines = applyEdits(intrinsic, edits, updateInitialFocal)
    for line in lines:
        logging.info(line)
    if not lines:
        logging.info("No edit")

    if not sfmDataIO.save(data, outputPath, sfmDataIO.ALL):
        raise RuntimeError(f"Cannot save SfMData {outputPath}")
