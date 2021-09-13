from datetime import datetime as dt
import datetime
from gps3 import gps3
import time

gps_socket = gps3.GPSDSocket()
data_stream = gps3.DataStream()
gps_socket.connect()
gps_socket.watch()

for new_data in gps_socket:
    if new_data:
        data_stream.unpack(new_data)
        if data_stream.TPV['time'] != "n/a":
            time_str = data_stream.TPV['time']
            epoch = dt.strptime(time_str[:-5] + time_str[-1:], "%Y-%m-%dT%H:%M:%S%z")
            time_epoch = time.perf_counter()
            break


def get_time() -> datetime:
    return epoch + datetime.timedelta(seconds=(time.perf_counter() - time_epoch))


def get_coordinates() -> tuple[float, float]:
    latitude = []
    longitude = []
    for new_data in gps_socket:
        if new_data:
            data_stream.unpack(new_data)
            if data_stream.TPV['lat'] != "n/a":
                latitude.append(float(data_stream.TPV['lat']))
                longitude.append(float(data_stream.TPV['lon']))
        if len(latitude) >= 5:
            return sum(latitude) / len(latitude), sum(longitude) / len(longitude)
