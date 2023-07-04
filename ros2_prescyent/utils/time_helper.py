"""functions to manipulate Pose objects"""
from typing import Tuple

from builtin_interfaces.msg import Time


DEFAULT_FLOAT = -1.


def update_time(time: Time, seconds: int, nanoseconds: int) -> Time:
    if nanoseconds + time.nanosec >= 1000000000:
        seconds += 1
        nanoseconds -= 1000000000
    return Time(sec=time.sec + seconds,
                nanosec=time.nanosec + nanoseconds)


def get_duration_from_frequency(frequency: int) -> Tuple[int, int]:
    """return a tuple of ints, seconds and nanosecond based on a frequency in Hz.
    Used for the timestamp creation of the predictions"""
    if frequency == 0: return (0, 0)
    return get_sec_nano_tuple_from_sec(1. / frequency)


def get_sec_nano_tuple_from_sec(sec: float) -> Tuple[int, int]:
    """from a float amount of seconds, returns seconds and nanosecs as
    a tuple of ints"""
    y = 1 if sec >= 0 else -1
    _i, _f = divmod(sec, y)
    return (int(_i) * y, int(_f * 10**9))
