#!/usr/bin/env python3
"""
image_convert — sensor_msgs/Image <-> numpy, without cv_bridge.

Why not cv_bridge
-----------------
cv_bridge's conversion runs through a compiled boost extension that is linked
against the numpy that ROS was built with (1.x on Humble). This project's
container installs numpy 2.x from pip on top of that, and the mismatch does not
fail loudly — `import cv_bridge` prints "AttributeError: _ARRAY_API not found"
and carries on, then the first `imgmsg_to_cv2` call SEGFAULTS the process.

An image message is a header plus a flat byte buffer plus an encoding string.
Reshaping that is a `np.frombuffer` away, so the C extension buys nothing here
and costs an ABI coupling that can take a node down mid-flight. zed_mimic_node
already reads depth this way for the same reason.

Only the encodings this stack actually moves are supported, and anything else
raises rather than guessing — a silently mis-decoded image is far worse than a
loud one.
"""

import numpy as np

from sensor_msgs.msg import Image


# encoding -> (numpy dtype, channels)
_LAYOUT = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "rgba8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    "8UC1": (np.uint8, 1),
    "8UC3": (np.uint8, 3),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def image_to_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> (H, W) or (H, W, C) array sharing no memory with msg.

    The result is a copy: the caller is free to write to it, and the message's
    buffer is free to be recycled.
    """
    try:
        dtype, channels = _LAYOUT[msg.encoding]
    except KeyError:
        raise ValueError(
            f"unsupported image encoding {msg.encoding!r}; "
            f"known: {sorted(_LAYOUT)}") from None

    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    flat = np.frombuffer(bytes(msg.data), dtype=dtype)

    # `step` is the row stride in BYTES and may exceed width*channels when the
    # publisher pads rows. Reshape by the stride, then trim, or the image comes
    # out sheared.
    stride = msg.step // dtype.itemsize
    expected = msg.height * stride
    if flat.size < expected:
        raise ValueError(
            f"image data is short: {flat.size} elements, expected {expected} "
            f"({msg.height}x{msg.step} bytes, encoding {msg.encoding})")

    rows = flat[:expected].reshape(msg.height, stride)
    used = msg.width * channels
    out = rows[:, :used].reshape(msg.height, msg.width, channels)
    return np.ascontiguousarray(out[:, :, 0] if channels == 1 else out)


def bgr_image_to_numpy(msg: Image) -> np.ndarray:
    """Like :func:`image_to_numpy`, but always returns BGR.

    OpenCV works in BGR; a driver publishing rgb8 would otherwise swap the pad's
    blue and red and the colour masks would find nothing.
    """
    arr = image_to_numpy(msg)
    if msg.encoding in ("rgb8", "rgba8"):
        arr = arr[:, :, ::-1] if arr.shape[2] == 3 else arr[:, :, [2, 1, 0, 3]]
        arr = np.ascontiguousarray(arr)
    if arr.ndim == 2:                       # mono -> 3 channels
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    return arr[:, :, :3]


def depth_image_to_numpy(msg: Image) -> np.ndarray:
    """Depth image -> float32 array in METRES.

    16UC1 is millimetres by ROS convention (that is what most real depth
    cameras publish); 32FC1 is already metres. Zeros in a 16UC1 image mean "no
    return" and become NaN, matching what the ZED and zed_mimic produce.
    """
    arr = image_to_numpy(msg)
    if msg.encoding in ("16UC1", "mono16"):
        out = arr.astype(np.float32) * 0.001
        out[arr == 0] = np.nan
        return out
    return arr.astype(np.float32, copy=False)


def numpy_to_image(array: np.ndarray, encoding: str = "bgr8") -> Image:
    """(H, W, 3) uint8 array -> sensor_msgs/Image. Header is left to the caller."""
    if array.ndim != 3 or array.shape[2] != 3 or array.dtype != np.uint8:
        raise ValueError(
            f"expected an (H, W, 3) uint8 array, got shape {array.shape} "
            f"dtype {array.dtype}")

    msg = Image()
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg
