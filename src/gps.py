import skyfield.toposlib
from datetime import datetime as dt, timezone
import datetime
from gps3 import gps3
import time
from skyfield.api import wgs84


class GPS(object):
    """GPS object to store GPS data"""
    # Disables gps if not in open area, which would cause a loop until a fix is acquired
    DEBUG_MODE = False

    def __init__(self):
        """Initialise GPS object with data from GPS receiver

        For performance, GPS data is only polled at initialisation
        """
        gps_socket = gps3.GPSDSocket()
        data_stream = gps3.DataStream()
        gps_socket.connect()
        gps_socket.watch()

        if self.DEBUG_MODE:
            self._lat = 57.0
            self._long = -7.0
        else:
            latitude = []
            longitude = []
            for new_data in gps_socket:
                if len(latitude) >= 5:
                    self._lat = sum(latitude) / len(latitude)
                    self._long = sum(longitude) / len(longitude)
                    break
                if new_data:
                    data_stream.unpack(new_data)
                    if data_stream.TPV['lat'] != "n/a":
                        latitude.append(float(data_stream.TPV['lat']))
                        longitude.append(float(data_stream.TPV['lon']))
                    if data_stream.TPV['time'] != "n/a":
                        time_str = data_stream.TPV['time']
                        self._epoch = dt.strptime(time_str[:-5] + time_str[-1:], "%Y-%m-%dT%H:%M:%S%z")
                        self._time_epoch = time.perf_counter()

    @property
    def location(self) -> skyfield.toposlib.GeographicPosition:
        """
        Gets the current location

        :return: The current location
        """
        return wgs84.latlon(self._lat, self._long)

    @property
    def time(self) -> datetime.datetime:
        """Calculates the current time by offsetting a stored GPS epoch

        :return: The current datetime
        """
        if self.DEBUG_MODE:
            return dt.now(timezone.utc)
        return self._epoch + datetime.timedelta(seconds=(time.perf_counter() - self._time_epoch))

gps = GPS()