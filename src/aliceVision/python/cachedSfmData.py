"""Cache of the content of an SfMData file.

`CachedSfmData` keeps the last SfMData it loaded and returns it again without reloading the file
as long as the file did not change (same path, same loading flags, same inode, modification and change times and size).
It is meant for interactive code (e.g. Meshroom node callbacks) that reads the same SfMData repeatedly.
"""

import logging
import os

from pyalicevision import sfmData, sfmDataIO


class CachedSfmData:
    """Load an SfMData file and keep it until the file or the requested content changes.

    The file's inode, modification and change times and size are checked, not only the path, because a file can be
    rewritten in place (e.g. a Meshroom node recomputed in its cache folder). The inode and change time catch
    rewrites that keep the size within the filesystem's modification time granularity (coarse on some NFS mounts).

    A failed load is cached too: the same unchanged file is not reloaded (nor logged) again.
    """

    def __init__(self):
        self._key = None
        self._data = None

    def clear(self):
        """Forget the cached SfMData."""
        self._key = None
        self._data = None

    def get(self, path, flags):
        """Return the SfMData stored in `path`, loaded with the `sfmDataIO` `flags`.

        Args:
            path (str): Path to the SfMData file.
            flags (int): sfmDataIO.ESfMData flags restricting what is loaded (e.g. sfmDataIO.INTRINSICS).

        Returns:
            sfmData.SfMData: The cached or freshly loaded SfMData, or None if the file is missing
            or cannot be loaded. The cached object itself is returned, not a copy: callers must not
            modify it.
        """
        if not path:
            self.clear()
            return None

        try:
            st = os.stat(path)
        except OSError:
            self.clear()
            return None

        key = (os.path.abspath(path), int(flags), st.st_ino, st.st_mtime_ns, st.st_ctime_ns, st.st_size)
        if key == self._key:
            return self._data

        data = sfmData.SfMData()
        try:
            loaded = sfmDataIO.load(data, path, flags)
            reason = ""
        except Exception as e:
            loaded = False
            reason = f": {e}"

        if not loaded:
            logging.warning(f"Cannot load SfMData file {path}{reason}")
            data = None

        self._key = key
        self._data = data
        return data
