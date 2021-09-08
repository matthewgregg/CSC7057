import jinja2
import aiohttp_jinja2
from aiohttp import web
import asyncio
import os.path
from datetime import datetime as dt, timedelta, timezone
import PIL
import numpy as np
from scipy import constants
from skyfield import sgp4lib, timelib as sftime
from skyfield.api import load, wgs84
from PIL import Image
from pydispatch import Dispatcher, Property
from rtlsdr import RtlSdr
from rtlsdr.rtlsdr import LibUSBError
import apt
import adafruit_mpu6050
import board
import motor
import shared
import gps
import elevation

MINIMUM_PASS_ANGLE = 30.0


class WebInterface(Dispatcher):
    value = Property()

    async def load_stop_web_interface(self, request):
        data = {
            'running': 0,
        }
        self.value = False
        return aiohttp_jinja2.render_template('index.html', request, data)

    async def load_start_web_interface(self, request):
        s, p = get_next_pass(MINIMUM_PASS_ANGLE)
        sat_name = s.name
        rise_time = dt.isoformat(p[0][0])
        set_time = dt.isoformat(p[2][0])
        if sdr_error == 0:
            self.value = True
        data = {
            'running': 1,
            'satellite': sat_name,
            'rise': rise_time,
            'set': set_time,
            'sdrerror': sdr_error
        }
        if shared.running.value and not shared.waiting.value:
            self.value = False
            data['redirect'] = True

        return aiohttp_jinja2.render_template('index.html', request, data)


def get_frequency(satellite: sgp4lib.EarthSatellite) -> int:
    if satellite.name == 'NOAA 15 [B]':
        return int(137620000)
    elif satellite.name == 'NOAA 18 [B]':
        return int(137912500)
    elif satellite.name == 'NOAA 19 [+]':
        return int(137100000)


def get_doppler_shift(satellite: sgp4lib.EarthSatellite) -> float:
    """
    :type satellite: skyfield.sgp4lib.EarthSatellite
    """
    t = load.timescale().now()
    pos = (satellite - location).at(t)
    (_, _, _, _, _, range_rate) = pos.frame_latlon_and_rates(location)
    return range_rate.m_per_s / constants.c * get_frequency(satellite)


def check_tle_epoch() -> None:
    tle_outdated = False
    if os.path.exists('noaa.txt'):
        with open('noaa.txt', "r+") as r:
            line = r.readlines()
            tle_year = int(line[1][18:20])
            tle_year += (1900 if tle_year >= 57 <= 99 else 2000)
            tle_day = float(line[1][20:32])
            tle_date = (dt(tle_year, 1, 1) + timedelta(tle_day - 1)).replace(tzinfo=timezone.utc)
            tle_outdated = (gps.get_time() - tle_date).total_seconds() > 7 * 24 * 3600

    if not os.path.exists('noaa.txt') or tle_outdated:
        import http.client as http
        conn = http.HTTPConnection("www.celestrak.com", timeout=5)
        try:
            conn.request("HEAD", "/")
            import requests
            noaa = requests.get('https://celestrak.com/NORAD/elements/noaa.txt')
            open('noaa.txt', 'wb').write(noaa.content)
        except:
            print('Could\'t download TLE sets. Will try again next time.')
            pass

        conn.close()


def get_passes(satellite: sgp4lib.EarthSatellite, altitude: float) -> list:
    ts = load.timescale()
    utc = gps.get_time()
    now = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
    end = ts.utc(utc.year, utc.month, utc.day + 1, utc.hour, utc.minute, utc.second)

    t, events = satellite.find_events(location, now, end, altitude)
    passes = []
    for time, event in zip(t, events):
        pos = (satellite - location).at(time)
        (alt, _, _) = pos.altaz()
        time = dt.strptime(sftime.Time.utc_iso(time), "%Y-%m-%dT%H:%M:%S%z")
        if event == 1 and len(passes) > 1 and passes[-1][1] == 1:
            if passes[-1][0] < time:
                passes[-1] = (time, event, alt)
        else:
            passes.append((time, event, alt))
    passes = np.array_split(passes, int(len(passes) / 3))
    return passes


def get_next_pass(altitude: float) -> tuple[sgp4lib.EarthSatellite, list]:
    passes = {}
    for s in sats:
        passes[s] = get_passes(s, altitude)

    sat = sats[0]
    closest = (sat, passes[sat][0])

    for s, p in passes.items():
        next_pass = p[0]
        rise = 0
        peak = 1
        setting = 2
        time = 0
        alt = 2

        if next_pass[rise][time] < closest[1][rise][time]:
            if next_pass[rise][time] < closest[1][setting][time]:
                closest = (s, next_pass)
            # if setting time overlaps with rise time of next, take the one with the highest peak elevation
            elif closest[1][peak][alt] > next_pass[peak][alt]:
                closest = (s, next_pass)
    return closest


def get_satellite_data(satellite):
    """
    :type satellite: skyfield.sgp4lib.EarthSatellite
    """

    ts = load.timescale()
    utc = gps.get_time()
    t = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
    geocentric = satellite.at(t)
    subpoint = wgs84.subpoint(geocentric)
    print('Latitude:', subpoint.latitude)
    print('Longitude:', subpoint.longitude)
    print('Height: {:.1f} km'.format(subpoint.antenna_elevation.km))
    pos = (satellite - location).at(t)
    alt, az, distance = pos.altaz()
    print('Altitude:', alt)
    print('Azimuth:', az)
    print('Distance: {:.1f} km'.format(distance.km))
    (_, _, sat_range, _, _, range_rate) = pos.frame_latlon_and_rates(location)
    print('Range: {:.1f} km'.format(sat_range.km))
    print('Range rate: {:.4f} km/s'.format(range_rate.km_per_s))


async def stream_decode_signal(p: list) -> None:
    print('streaming method called')
    finish_time = p[2][0]
    output = []
    sample_rate = sdr.sample_rate
    async for samples in sdr.stream(5 * int(sample_rate)):
        if gps.get_time() > finish_time or not shared.running.value:
            print('doppler stopped at ' + finish_time)
        sdr.stop()
        decoded = apt.decode(sample_rate, samples)
        output.extend(decoded)
        PIL.Image.fromarray(np.array(output)).save('media/image.png')


async def set_frequency(frequency: int, satellite: sgp4lib.EarthSatellite) -> None:
    sdr.center_freq = int(frequency + get_doppler_shift(satellite))


def signaltonoise(a: np.ndarray, axis: int = 0, ddof: int = 0) -> np.ndarray:
    mx = np.amax(a)
    a = np.divide(a, mx)
    a = np.square(a)
    a = np.asanyarray(a)
    m = a.mean(axis)
    sd = a.std(axis=axis, ddof=ddof)
    return np.where(sd == 0, 0, m / sd)


async def motor_controller(s: sgp4lib.EarthSatellite, p: list) -> None:
    print('motor method called')
    finish_time = p[2][0]
    # negative ratio as final gear moves in the opposite direction to spindle
    elevation_motor_ratio = -120
    elevation_motor = [24, 25, 8, 7]
    azimuth_motor_ratio = -3
    azimuth_motor = [12, 16, 20, 21]
    # TODO azimuth motor
    # wait for accelerometer data
    await asyncio.sleep(1)
    while True:
        antenna_elevation, _ = elevation.elevation.get_adjusted_elevation()
        if gps.get_time() > finish_time or not shared.running.value:
            print('motor stopped')
            print('resetting elevation motor')
            elevation_angle_delta = (90 - antenna_elevation) * elevation_motor_ratio
            print('moving', antenna_elevation)
            await motor.rotate(elevation_motor, elevation_angle_delta, 0)
            break

        ts = load.timescale()
        utc = gps.get_time()
        t = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
        pos = (s - location).at(t)
        sat_elevation, sat_azimuth, _ = pos.altaz()
        elevation_angle_delta = (abs(sat_elevation.degrees) - antenna_elevation) * elevation_motor_ratio
        print('arm ', antenna_elevation)
        print('sat ', abs(sat_elevation.degrees))
        print('delta', abs(sat_elevation.degrees) - antenna_elevation)
        await asyncio.gather(
            asyncio.sleep(10),
            motor.rotate(elevation_motor, elevation_angle_delta, 0)
        )


async def doppler_task_controller(s: sgp4lib.EarthSatellite, p: list) -> None:
    print('doppler method called')
    frequency = get_frequency(s)
    finish_time = p[2][0]
    while True:
        if gps.get_time() > finish_time or not shared.running.value:
            print('doppler stopped at ' + finish_time)
            break
        await asyncio.gather(
            asyncio.sleep(10),
            set_frequency(frequency, s)
        )


async def wait_for_pass() -> tuple[sgp4lib.EarthSatellite, list]:
    print('waiting for pass')
    shared.waiting.value = True
    s, p = get_next_pass(MINIMUM_PASS_ANGLE)
    while p[0][0] > gps.get_time():
        await asyncio.sleep(0.5)
        if not shared.running.value:
            break
    shared.waiting.value = False
    return s, p


async def task_listener() -> None:
    while True:
        (_, shared.running.value), _ = await event
        print('running val changed to ', shared.running.value)


async def task_dispatcher() -> None:
    while True:
        (_, value), _ = await event
        if value:
            shared.running.value = True
            # TODO revert this when not testing
            s, p = get_next_pass(MINIMUM_PASS_ANGLE)#await wait_for_pass()
            if shared.running.value:

                coro = [stream_decode_signal(p),
                        doppler_task_controller(s, p),
                        motor_controller(s, p),
                        elevation.elevation.stream_readings(p)]
                await asyncio.gather(*coro, return_exceptions=True)
            print('done')
            if os.path.exists('media/image.png'):
                os.rename('media/image.png', 'media/previous/' + str(gps.get_time()) + '.png')


async def background_task_setup(app) -> None:
    asyncio.create_task(task_listener())
    asyncio.create_task(task_dispatcher())


if __name__ == '__main__':
    check_tle_epoch()
    all_sats = load.tle_file('noaa.txt')
    by_name = {sat.name: sat for sat in all_sats}
    noaa_15 = by_name['NOAA 15 [B]']
    noaa_18 = by_name['NOAA 18 [B]']
    noaa_19 = by_name['NOAA 19 [+]']
    sats = [noaa_15, noaa_18, noaa_19]
    lat, long = gps.get_coordinates()
    location = wgs84.latlon(lat, long)
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    try:
        sdr = RtlSdr()
        # super sample to increase analog to digital conversion resolution
        sdr.sample_rate = 2.08e6
        # APT overall bandwidth is 38.8 kHz per Carson bandwidth rule - 2 * (17 + 2.4)
        sdr.bandwidth = 38800
        sdr_error = 0
    except LibUSBError:
        sdr_error = 1

    emitter = WebInterface()
    event = emitter.get_dispatcher_event('value')
    app = web.Application()
    app.on_startup.append(background_task_setup)
    app.add_routes([web.get('/', emitter.load_stop_web_interface)])
    app.add_routes([web.get('/running', emitter.load_start_web_interface)])
    app.add_routes([web.static('/media', os.getcwd() + '/media/')])
    aiohttp_jinja2.setup(app, loader=jinja2.FileSystemLoader(os.getcwd() + '/templates/'))
    web.run_app(app)
