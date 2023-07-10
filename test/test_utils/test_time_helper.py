from unittest import TestCase

from ros2_prescyent.utils.time_helper import (
    get_sec_nano_tuple_from_sec,
    get_duration_from_frequency,
    update_time,
    Time,
)


class TimeHelperTest(TestCase):
    def test_zero_sec_to_tuple(self):
        self.assertEquals((0, 0), get_sec_nano_tuple_from_sec(0.0))
        self.assertEquals((1, 0), get_sec_nano_tuple_from_sec(1.0))
        self.assertEquals((0, 100), get_sec_nano_tuple_from_sec(0.0000001))
        self.assertEquals((-1, -100), get_sec_nano_tuple_from_sec(-1.0000001))

    def test_time_update(self):
        time = Time(sec=1, nanosec=1000)
        new_time = update_time(time, 0, 0)
        self.assertEquals(time, new_time)
        new_time = update_time(time, 0, 1000)
        self.assertEquals(Time(sec=1, nanosec=2000), new_time)
        new_time = update_time(time, 0, 1000000000)
        self.assertEquals(Time(sec=2, nanosec=1000), new_time)
        new_time = update_time(time, 0, 999999000)
        self.assertEquals(Time(sec=2, nanosec=0), new_time)
        new_time = update_time(time, 5, 999999000)
        self.assertEquals(Time(sec=7, nanosec=0), new_time)
        new_time = update_time(time, -1, -1000)
        self.assertEquals(Time(sec=0, nanosec=0), new_time)

    def test_get_duration_float_from_frequency(self):
        self.assertEquals((0, 0), get_duration_from_frequency(0))
        self.assertEquals((1, 0), get_duration_from_frequency(1))
        self.assertEquals((0, 10000000), get_duration_from_frequency(100))
