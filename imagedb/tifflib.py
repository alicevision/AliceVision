"""Minimal little-endian TIFF/DNG/DCP writer (standard library only).

The IFD is always written right after the 8-byte header, because the DCP reader of AliceVision
(image/dcp.cpp, DCPProfile::Load) reads the tag count at offset 8 without seeking to the IFD offset.
"""
import struct

BYTE, ASCII, SHORT, LONG, RATIONAL, SBYTE, UNDEFINED, SSHORT, SLONG, SRATIONAL, FLOAT, DOUBLE = range(1, 13)
_SIZES = {BYTE: 1, ASCII: 1, SHORT: 2, LONG: 4, RATIONAL: 8, SBYTE: 1, UNDEFINED: 1, SSHORT: 2, SLONG: 4,
          SRATIONAL: 8, FLOAT: 4, DOUBLE: 8}
_FMT = {BYTE: "B", SHORT: "H", LONG: "I", SBYTE: "b", UNDEFINED: "B", SSHORT: "h", SLONG: "i", FLOAT: "f",
        DOUBLE: "d"}


def _rational(value, signed, den=10000):
    num = int(round(value * den))
    return struct.pack("<ii" if signed else "<II", num, den)


def _encode(ttype, values):
    """Return (count, payload bytes) for a tag value."""
    if ttype == ASCII:
        data = values.encode("latin-1") + b"\0"
        return len(data), data
    if not isinstance(values, (list, tuple)):
        values = [values]
    if ttype in (RATIONAL, SRATIONAL):
        data = b"".join(_rational(v, ttype == SRATIONAL) for v in values)
    else:
        data = struct.pack("<" + _FMT[ttype] * len(values), *values)
    return len(values), data


def write_tiff(path, tags, magic=42, pixel_data=None, strip_tag=273):
    """Write a single-IFD TIFF.

    tags: list of (tag, type, values). If pixel_data is given, its offset is stored in `strip_tag`
    (StripOffsets) — the caller must include that tag with a placeholder value of 0.
    magic: 42 for TIFF/DNG, 0x4352 ("RC") for DCP.
    """
    tags = sorted(tags, key=lambda t: t[0])
    n = len(tags)
    ifd_size = 2 + 12 * n + 4
    data_offset = 8 + ifd_size
    entries = []
    extra = bytearray()
    payloads = []
    for tag, ttype, values in tags:
        count, payload = _encode(ttype, values)
        payloads.append((tag, ttype, count, payload))
    # out-of-line data first, pixel data last
    offsets = {}
    for tag, ttype, count, payload in payloads:
        if len(payload) > 4:
            offsets[tag] = data_offset + len(extra)
            extra += payload
            if len(extra) % 2:
                extra += b"\0"
    pixel_offset = data_offset + len(extra)
    for tag, ttype, count, payload in payloads:
        if tag == strip_tag and pixel_data is not None:
            payload = struct.pack("<I", pixel_offset)
            ttype, count = LONG, 1
        if len(payload) > 4:
            value = struct.pack("<I", offsets[tag])
        else:
            value = payload.ljust(4, b"\0")
        entries.append(struct.pack("<HHI", tag, ttype, count) + value)
    out = bytearray(b"II" + struct.pack("<HI", magic, 8))
    out += struct.pack("<H", n) + b"".join(entries) + struct.pack("<I", 0)
    out += extra
    if pixel_data is not None:
        out += pixel_data
    with open(path, "wb") as f:
        f.write(out)
