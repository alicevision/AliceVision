"""Dynamic parallelization strategies for AliceVision processing nodes.

This module provides classes that compute chunk sizes for parallelizing
AliceVision pipeline nodes based on the number of views in an SfMData file.
Each class implements a `__call__` method that returns the number of
parallel chunks a node should be split into.
"""

from meshroom.core import desc

class DynamicViewsSize(object):
    """Compute parallelization size based on the total number of views.

    Returns the total number of views found in the SfMData file referenced
    by the given node parameter, with a minimum of 1.

    Args:
        param: Name of the node attribute that holds the path to the SfMData file.
    """

    def __init__(self, param):
        self._param = param

    def __call__(self, node):
        """Compute the number of chunks from the total view count.

        Args:
            node: The processing node whose attribute *param* points to an SfMData file.

        Returns:
            int: The number of views in the SfMData file (at least 1).

        Raises:
            RuntimeError: If the SfMData file cannot be loaded.
        """

        from pyalicevision import sfmData
        from pyalicevision import sfmDataIO

        param = node.attribute(self._param)

        data = sfmData.SfMData()
        ret = sfmDataIO.load(data, param.value, sfmDataIO.VIEWS)
        if not ret:
            raise RuntimeError(f"Failed to load file : {param.value}")
        
        return max(1, len(data.getViews()))
    
class DynamicReconstructedViewsSize(object):
    """Compute parallelization size based on the number of reconstructed views.

    Only views that have both a defined pose and intrinsic parameters are
    counted, with a minimum of 1.

    Args:
        param: Name of the node attribute that holds the path to the SfMData file.
    """

    def __init__(self, param):
        self._param = param

    def __call__(self, node):
        """Compute the number of chunks from the reconstructed view count.

        A view is considered reconstructed when it has both a valid pose and
        valid intrinsic parameters defined in the SfMData file.

        Args:
            node: The processing node whose attribute *param* points to an SfMData file.

        Returns:
            int: The number of reconstructed views (at least 1).

        Raises:
            RuntimeError: If the SfMData file cannot be loaded.
        """

        from pyalicevision import sfmData
        from pyalicevision import sfmDataIO

        param = node.attribute(self._param)

        data = sfmData.SfMData()
        ret = sfmDataIO.load(data, param.value, sfmDataIO.VIEWS | sfmDataIO.EXTRINSICS | sfmDataIO.INTRINSICS)
        if not ret:
            raise RuntimeError(f"Failed to load file : {param.value}")

        size = 0
        for viewId in data.getViews():
            if data.isPoseAndIntrinsicDefined(viewId):
                size = size + 1

        return max(1, size)
    
class DynamicDividedViewsSize(object):
    """Compute parallelization size by dividing the total view count.

    Returns the total number of views divided by a configurable integer
    divider, with a minimum of 1 for both the divider and the result.

    Args:
        param: Name of the node attribute that holds the path to the SfMData file.
        divider: Name of the node attribute (must be an ``IntParam``) used as
            the divisor.
    """

    def __init__(self, param, divider):

        self._param = param
        self._divider = divider

    def __call__(self, node):
        """Compute the number of chunks by dividing the view count.

        Args:
            node: The processing node whose attributes *param* and *divider*
                provide the SfMData path and the divisor value respectively.

        Returns:
            int: ``max(1, total_views / divider_value)``.

        Raises:
            RuntimeError: If the divider attribute is not an ``IntParam`` or
                if the SfMData file cannot be loaded.
        """

        from pyalicevision import sfmData
        from pyalicevision import sfmDataIO
        import math

        param = node.attribute(self._param)
        divider = node.attribute(self._divider)

        if not isinstance(divider.desc, desc.IntParam):
            raise RuntimeError(f"Divider object is not a number.")
        
        data = sfmData.SfMData()
        ret = sfmDataIO.load(data, param.value, sfmDataIO.VIEWS)
        if not ret:
            raise RuntimeError(f"Failed to load file : {param.value}")
        
        valDivider = max(1, divider.value)
        size = math.ceil(len(data.getViews()) / valDivider)

        return max(1, size)