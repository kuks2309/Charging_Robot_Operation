"""Utility functions for camera device detection on Linux sysfs."""

from typing import Optional
import os


def detect_arducam_index(keyword: str = "FHD Camera") -> Optional[int]:
    """Detect the video device index for an ArduCam (or similar) capture node.

    Scans /sys/class/video4linux/videoN entries and returns the first N where:
    1. The device name contains ``keyword`` (substring match).
    2. The device index file contains ``0`` (capture node, not metadata node).

    Parameters
    ----------
    keyword:
        Substring to match against the sysfs ``name`` file.  Defaults to
        ``"FHD Camera"``.

    Returns
    -------
    int or None
        The integer N of the first matching videoN node, or None if no match
        is found or the sysfs path does not exist (e.g. non-Linux systems).
    """
    sysfs_root = "/sys/class/video4linux"
    if not os.path.isdir(sysfs_root):
        return None

    entries = []
    try:
        for entry in os.listdir(sysfs_root):
            if entry.startswith("video"):
                suffix = entry[len("video"):]
                if suffix.isdigit():
                    entries.append((int(suffix), entry))
    except OSError:
        return None

    for n, entry in sorted(entries):
        base = os.path.join(sysfs_root, entry)
        name_path = os.path.join(base, "name")
        index_path = os.path.join(base, "index")

        try:
            with open(name_path, "r") as f:
                name = f.read().strip()
        except OSError:
            continue

        if keyword not in name:
            continue

        try:
            with open(index_path, "r") as f:
                idx = f.read().strip()
        except OSError:
            continue

        if idx == "0":
            return n

    return None
