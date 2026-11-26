__version__ = "1.0"

import contextlib
import functools
import logging

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL


# AUTO and the field names are duplicated in pyalicevision.intrinsicsEditing
# (src/aliceVision/python/intrinsicsEditing.py), which cannot be imported at file level here: keep them in sync.
AUTO = "auto"

# Flat field name (as used by pyalicevision.intrinsicsEditing) -> attribute path in the node
FIELD_ATTRIBUTES = {
    "focalLength": ("focalLength",),
    "pixelRatio": ("pixelRatio",),
    "centerOffsetX": ("centerOffset", "x"),
    "centerOffsetY": ("centerOffset", "y"),
    "sensorWidth": ("sensorWidth",),
    "sensorHeight": ("sensorHeight",),
    "circleCenterX": ("circleCenter", "x"),
    "circleCenterY": ("circleCenter", "y"),
    "circleRadius": ("circleRadius",),
}

# Default of each field and of its snapshot: values a real camera does not have, so that a field still at
# its default visibly means "not filled from the input".
FIELD_DEFAULTS = {
    "focalLength": 0.0,
    "pixelRatio": 0.0,
    "centerOffsetX": -100000.0,
    "centerOffsetY": -100000.0,
    "sensorWidth": 0.0,
    "sensorHeight": 0.0,
    "circleCenterX": -100000.0,
    "circleCenterY": -100000.0,
    "circleRadius": 0.0,
}

# Lock field name (as used by pyalicevision.intrinsicsEditing) -> attribute name in the node.
# "locked" cannot be an attribute name: Node.locked is a Meshroom Node property.
LOCK_ATTRIBUTES = {
    "locked": "intrinsicLocked",
    "scaleLocked": "scaleLocked",
    "offsetLocked": "offsetLocked",
    "ratioLocked": "ratioLocked",
}
LOCK_VALUES = ["keep", "locked", "unlocked"]

CAMERA_TYPES = ["pinhole", "equirectangular", "equidistant"]


def _isEquidistant(node):
    return node.cameraType.value == "equidistant"


def _snapshotParam(name):
    return desc.FloatParam(
        name=name,
        label=name,
        description="Value the field was filled with from the input.",
        value=FIELD_DEFAULTS[name],
        range=None,
    )


def _batchedUpdate(node):
    """Group attribute writes into a single graph update: each write otherwise updates the whole graph."""
    if node.graph is None:
        return contextlib.nullcontext()
    from meshroom.core.graph import GraphModification
    return GraphModification(node.graph)


def _logErrors(callback):
    """Log the errors of a node callback instead of raising them.

    Callbacks run inside Meshroom's attribute change handling: an error raised there would skip the rest of it
    (e.g. the notification of the downstream nodes). The fields are left as they are.
    """
    @functools.wraps(callback)
    def wrapper(self, node):
        try:
            callback(self, node)
        except Exception:
            logging.exception(f"{node.name}: {callback.__name__} failed")
    return wrapper


def _logLevel(verboseLevel):
    return {"trace": logging.DEBUG, "fatal": logging.CRITICAL}.get(verboseLevel, verboseLevel.upper())


class IntrinsicsEditing(desc.Node):
    """
Edit by hand the parameters of one camera intrinsic of an SfMData.

Select the intrinsic with "Intrinsic ID" ("auto" stands for the only intrinsic of the input).
The fields are filled with the selected intrinsic's values from the input; change the ones to edit.
Only the fields you changed are applied: the others keep the input's value, even if the input changes later.
Changing the selected intrinsic refills all the fields and discards the changes.
The refills are not on the undo stack: undoing an "Intrinsic ID" change does not bring back the discarded changes.

The focal length is edited in mm and stored in pixels: focal (px) = focal (mm) * max(image width, image height) / sensor width.
Changing the sensor width keeps the focal length in mm, so it changes the focal length in pixels, i.e. the field of view.
On an SfMData that is already reconstructed, this changes the estimated camera: to keep the field of view, scale
the focal length in mm by the same ratio as the sensor width.

The camera type is not modified: the computation fails if "Camera Type" differs from the input intrinsic's type.
Distortion and undistortion are not modified.
"""

    size = desc.DynamicNodeSize("input")
    category = "Utils"

    inputs = [
        desc.File(
            name="input",
            label="Input SfMData",
            description="Input SfMData file.",
            value="",
        ),
        desc.ChoiceParam(
            name="intrinsicId",
            label="Intrinsic ID",
            description="Intrinsic to edit. 'auto' stands for the only intrinsic of the input "
                        "(the computation fails if the input has several intrinsics).",
            values=[AUTO],
            value=AUTO,
            exclusive=True,
            # The ID list is saved with the project, so that it is not rebuilt by reading the input
            # each time the graph is loaded or updated.
            saveValuesOverride=True,
        ),
        desc.PushButtonParam(
            name="refreshFromInput",
            label="Refresh From Input",
            description="Reload the intrinsic IDs, set the camera type to the input intrinsic's type and refill "
                        "the fields you did not change from the input (e.g. after the upstream node was computed).",
            invalidate=False,
        ),
        desc.ChoiceParam(
            name="cameraType",
            label="Camera Type",
            description="Expected camera type of the edited intrinsic. It is not modified: "
                        "the computation fails if it differs from the input intrinsic's type.",
            values=CAMERA_TYPES,
            value="pinhole",
        ),
        desc.FloatParam(
            name="focalLength",
            label="Focal Length",
            description="Focal length (in mm).\n"
                        "Converted to pixels with the sensor width: focal (px) = focal (mm) * max(image width, image height) / sensor width.",
            value=FIELD_DEFAULTS["focalLength"],
            range=None,
        ),
        desc.FloatParam(
            name="pixelRatio",
            label="Pixel Ratio",
            description="Ratio between the pixel width and the pixel height.",
            value=FIELD_DEFAULTS["pixelRatio"],
            range=None,
        ),
        desc.GroupAttribute(
            name="centerOffset",
            label="Principal Point Offset (from center)",
            description="Offset of the principal point from the image center (in pixels).",
            items=[
                desc.FloatParam(
                    name="x",
                    label="x",
                    description="Horizontal offset (in pixels).",
                    value=FIELD_DEFAULTS["centerOffsetX"],
                    range=None,
                ),
                desc.FloatParam(
                    name="y",
                    label="y",
                    description="Vertical offset (in pixels).",
                    value=FIELD_DEFAULTS["centerOffsetY"],
                    range=None,
                ),
            ],
        ),
        desc.FloatParam(
            name="sensorWidth",
            label="Sensor Width",
            description="Sensor width (in mm).\n"
                        "The focal length is kept in mm, so changing the sensor width changes the focal length in pixels\n"
                        "(i.e. the field of view). To keep the field of view, scale the focal length in mm by the same ratio.",
            value=FIELD_DEFAULTS["sensorWidth"],
            range=None,
        ),
        desc.FloatParam(
            name="sensorHeight",
            label="Sensor Height",
            description="Sensor height (in mm).\n"
                        "It does not change the focal length in pixels.",
            value=FIELD_DEFAULTS["sensorHeight"],
            range=None,
        ),
        desc.GroupAttribute(
            name="circleCenter",
            label="Circle Center",
            description="Center of the fisheye circle (in pixels). Equidistant cameras only.",
            items=[
                desc.FloatParam(
                    name="x",
                    label="x",
                    description="Horizontal position (in pixels).",
                    value=FIELD_DEFAULTS["circleCenterX"],
                    range=None,
                ),
                desc.FloatParam(
                    name="y",
                    label="y",
                    description="Vertical position (in pixels).",
                    value=FIELD_DEFAULTS["circleCenterY"],
                    range=None,
                ),
            ],
            enabled=_isEquidistant,
        ),
        desc.FloatParam(
            name="circleRadius",
            label="Circle Radius",
            description="Radius of the fisheye circle (in pixels). Equidistant cameras only.",
            value=FIELD_DEFAULTS["circleRadius"],
            range=None,
            enabled=_isEquidistant,
        ),
        desc.ChoiceParam(
            name="intrinsicLocked",
            label="Locked",
            description="Lock all the intrinsic parameters during estimation.\n"
                        " - keep: keep the input's state.\n"
                        " - locked / unlocked: force the state.",
            values=LOCK_VALUES,
            value="keep",
        ),
        desc.ChoiceParam(
            name="scaleLocked",
            label="Focal Length Locked",
            description="Lock the focal length during estimation.\n"
                        " - keep: keep the input's state.\n"
                        " - locked / unlocked: force the state.",
            values=LOCK_VALUES,
            value="keep",
        ),
        desc.ChoiceParam(
            name="offsetLocked",
            label="Principal Point Locked",
            description="Lock the principal point during estimation.\n"
                        " - keep: keep the input's state.\n"
                        " - locked / unlocked: force the state.",
            values=LOCK_VALUES,
            value="keep",
        ),
        desc.ChoiceParam(
            name="ratioLocked",
            label="Pixel Ratio Locked",
            description="Lock the pixel ratio during estimation.\n"
                        " - keep: keep the input's state.\n"
                        " - locked / unlocked: force the state.",
            values=LOCK_VALUES,
            value="keep",
        ),
        desc.BoolParam(
            name="updateInitialFocal",
            label="Use Focal Length Prior",
            description="Use the focal length value as a prior during the optimization: the initial focal length\n"
                        "limits the range of possible focal length values in the optimization.\n"
                        "If enabled, the initial focal length is set to the focal length whenever the focal length\n"
                        "in pixels is recomputed (focal length, pixel ratio or sensor width edit).\n"
                        "If disabled, the initial focal length is set to -1: the focal length will not be bounded.",
            value=True,
        ),
        desc.GroupAttribute(
            name="snapshot",
            label="Snapshot",
            description="Values the fields were filled with from the input: a field is applied only if it differs "
                        "from its snapshot value. Written by the node.",
            items=[_snapshotParam(name) for name in FIELD_ATTRIBUTES],
            # visible=False only hides the graph pin; advanced hides it from the Node Editor by default.
            # Not disabled: disabled attributes are left out of the node UID.
            visible=False,
            advanced=True,
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        ),
    ]

    outputs = [
        desc.File(
            name="output",
            label="Output SfMData",
            description="Output SfMData file.",
            value="{nodeCacheFolder}/sfmData.usda",
        ),
    ]

    def _reader(self):
        cache = getattr(self, "_cache", None)
        if cache is None:
            from pyalicevision import cachedSfmData
            cache = cachedSfmData.CachedSfmData()
            self._cache = cache
        return cache

    def _readInput(self, node):
        from pyalicevision import sfmDataIO
        return self._reader().get(node.input.value, sfmDataIO.INTRINSICS)

    def _readSelected(self, node):
        """(camera type, {field: value}) of the selected intrinsic, or None if it cannot be read."""
        from pyalicevision import intrinsicsEditing
        data = self._readInput(node)
        if data is None:
            return None
        return intrinsicsEditing.readIntrinsic(data, node.intrinsicId.value)

    def _refreshIdList(self, node):
        data = self._readInput(node)
        if data is None:
            # Keep the current list (and so the selected ID) until the input can be read, e.g. once computed
            return
        values = [AUTO] + [str(i) for i in sorted(data.getIntrinsics().keys())]
        node.intrinsicId.setValues(values)
        if node.intrinsicId.value not in values:
            # Triggers onIntrinsicIdChanged: the fields are refilled from the new selection
            logging.warning(f"{node.name}: intrinsic ID '{node.intrinsicId.value}' is not in the input, "
                            f"reset to '{AUTO}'")
            node.intrinsicId.value = AUTO

    @staticmethod
    def _field(node, name):
        attribute = node
        for part in FIELD_ATTRIBUTES[name]:
            attribute = getattr(attribute, part)
        return attribute

    @staticmethod
    def _snapshot(node, name):
        return getattr(node.snapshot, name)

    def _fill(self, node, values):
        """Write `values` (missing fields: defaults) to the fields and to the snapshot, reset the locks."""
        for name in FIELD_ATTRIBUTES:
            value = values.get(name, FIELD_DEFAULTS[name])
            self._field(node, name).value = value
            self._snapshot(node, name).value = value
        for attributeName in LOCK_ATTRIBUTES.values():
            node.attribute(attributeName).value = "keep"

    def _isFilled(self, node):
        """Whether the fields were already filled from an input (snapshot not at its defaults)."""
        return any(self._snapshot(node, name).value != FIELD_DEFAULTS[name] for name in FIELD_ATTRIBUTES)

    def _refreshFromInput(self, node, syncCameraType):
        """Rebuild the intrinsic ID list and refill the fields the user did not change.

        The camera type is set to the input intrinsic's type on the first fill, or if `syncCameraType` is set.
        """
        from pyalicevision import intrinsicsEditing

        self._refreshIdList(node)
        selected = self._readSelected(node)
        if selected is None:
            return
        typeStr, values = selected
        if (syncCameraType or not self._isFilled(node)) and typeStr in CAMERA_TYPES:
            node.cameraType.value = typeStr
        # Refill the fields the user did not change (still equal to their snapshot).
        # Fields the input camera type does not have (e.g. circle on a pinhole) go back to their default.
        for name in FIELD_ATTRIBUTES:
            value = values.get(name, FIELD_DEFAULTS[name])
            field = self._field(node, name)
            snapshot = self._snapshot(node, name)
            if intrinsicsEditing.isClose(field.value, snapshot.value):
                field.value = value
                snapshot.value = value

    @_logErrors
    def onNodeCreated(self, node):
        # Fields are not reset: values preset in a pipeline template are kept
        self._refreshIdList(node)

    @_logErrors
    def onInputChanged(self, node):
        # The camera type is not synced: it guards against applying the edits to an intrinsic of another type
        with _batchedUpdate(node):
            self._refreshFromInput(node, syncCameraType=False)

    @_logErrors
    def onRefreshFromInputClicked(self, node):
        with _batchedUpdate(node):
            self._refreshFromInput(node, syncCameraType=True)

    @_logErrors
    def onIntrinsicIdChanged(self, node):
        # The user's changes are discarded: all the fields are refilled
        selected = self._readSelected(node)
        with _batchedUpdate(node):
            if selected is None:
                self._fill(node, {})
                return
            typeStr, values = selected
            self._fill(node, values)
            if typeStr in CAMERA_TYPES:
                node.cameraType.value = typeStr

    def editArguments(self, node):
        """Keyword arguments of pyalicevision.intrinsicsEditing.editSfmData, except outputPath."""
        return {
            "inputPath": node.input.value,
            "intrinsicId": node.intrinsicId.value,
            "cameraType": node.cameraType.value,
            "fields": {name: float(self._field(node, name).value) for name in FIELD_ATTRIBUTES},
            "snapshot": {name: float(self._snapshot(node, name).value) for name in FIELD_ATTRIBUTES},
            "locks": {name: node.attribute(attributeName).value for name, attributeName in LOCK_ATTRIBUTES.items()},
            "updateInitialFocal": node.updateInitialFocal.value,
            "defaults": dict(FIELD_DEFAULTS),
        }

    def processChunk(self, chunk):
        from pyalicevision import intrinsicsEditing, system

        verboseLevel = chunk.node.verboseLevel.value
        logging.getLogger().setLevel(_logLevel(verboseLevel))
        system.initialize_alicevision_logger(verboseLevel)
        intrinsicsEditing.editSfmData(outputPath=chunk.node.output.value, **self.editArguments(chunk.node))
