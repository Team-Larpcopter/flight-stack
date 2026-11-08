"""Topic filter for recording: every topic except raw image streams."""

IMAGE_MODES = ('compressed', 'none', 'raw')

# Kept even inside an image namespace, per image mode.
_KEEP_SUFFIXES = {
    'compressed': ('/camera_info', '/compressed', '/compressedDepth'),
    'none': ('/camera_info',),
}


def exclude_regex(images='compressed', extra=''):
    """Return the regex for ``ros2 bag record -x``, or '' to exclude nothing.

    A topic counts as an image when a name segment is ``image`` or starts with
    ``image_`` (``/stereo/left/image``, ``/camera/depth/image_raw``, ...). With
    ``compressed``, image topics are skipped unless they end in
    ``/camera_info``, ``/compressed`` or ``/compressedDepth``; ``none`` also
    skips compressed images; ``raw`` keeps everything. The pattern gives the
    same result whether rosbag2 searches or full-matches it.
    """
    if images not in IMAGE_MODES:
        raise ValueError(f'images must be one of {", ".join(IMAGE_MODES)}, not {images!r}')
    parts = []
    if images != 'raw':
        keep = '|'.join(_KEEP_SUFFIXES[images])
        parts.append(f'^(?!.*({keep})$).*/image(_[^/]*)?(/.*)?$')
    if extra:
        parts.append(extra)
    if len(parts) == 1:
        return parts[0]
    return '|'.join(f'({part})' for part in parts)
