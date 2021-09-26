import jinja2
import aiohttp_jinja2
import skyfield
from RPi import GPIO
from aiohttp import web
import asyncio
import os.path
from datetime import datetime as dt, timedelta, timezone
import numpy as np
from scipy import constants
from skyfield import sgp4lib, timelib as sftime
from skyfield.api import load, wgs84
from pydispatch import Dispatcher, Property
from rtlsdr import RtlSdr
from rtlsdr.rtlsdr import LibUSBError
import json
import smbus
import apt
import motor
import shared
import gps
import imu

MINIMUM_PASS_ANGLE = 30.0

class WebInterface(Dispatcher):
    value = Property()

    async def load_stop_web_interface(self, request):
        """
        Loads the stopped web interface
        :param request: The HTTP request
        :return: The HTML template to be displayed
        """
        data = {
            'running': 0,
        }
        self.value = False
        return aiohttp_jinja2.render_template('index.html', request, data)


    async def load_start_web_interface(self, request):
        """
        Loads the started web interface
        :param request: The HTTP request
        :return: The HTML template to be displayed
        """
        data = await request.post()
        shared.filter_type = data['filter-type']
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
    """
    Get the nominal APT frequency of a NOAA satellite
    :param satellite: The satellite to get the nominal frequency of
    :return: The nominal APT frequency
    :rtype: int
    """
    if satellite.name == 'NOAA 15 [B]':
        return int(137620000)
    elif satellite.name == 'NOAA 18 [B]':
        return int(137912500)
    elif satellite.name == 'NOAA 19 [+]':
        return int(137100000)
    else:
        raise ValueError("Not a NOAA APT satellite")


def get_doppler_shift(satellite: sgp4lib.EarthSatellite) -> float:
    """
    Calculates the Doppler shift based on the range rate of a satellite
    :param satellite: The satellite to calculate the Doppler shift for
    :type satellite: skyfield.sgp4lib.EarthSatellite
    """
    t = load.timescale().now()
    pos = (satellite - gps.gps.location).at(t)
    (_, _, _, _, _, range_rate) = pos.frame_latlon_and_rates(gps.gps.location)
    return range_rate.m_per_s / constants.c * get_frequency(satellite)


def check_tle_epoch(file) -> bool:
    """
    Checks whether the Two Line Element Set is outdated
    :return: False if outdated
    """
    tle_outdated = False
    if os.path.exists(file):
        with open(file, "r+") as r:
            line = r.readlines()
            tle_year = int(line[1][18:20])
            tle_year += (1900 if tle_year >= 57 <= 99 else 2000)
            tle_day = float(line[1][20:32])
            tle_date = (dt(tle_year, 1, 1) + timedelta(tle_day - 1)).replace(tzinfo=timezone.utc)
            tle_outdated = (gps.gps.time - tle_date).total_seconds() > 7 * 24 * 3600


    if not os.path.exists(file) or tle_outdated:
        return False
    else:
        return True


def update_tle(file) -> None:
    """
    Updates TLE file
    :param file: the filename of the TLE file
    :return: None
    """
    import http.client as http
    conn = http.HTTPConnection("www.celestrak.com", timeout=5)
    try:
        conn.request("HEAD", "/")
        import requests
        noaa = requests.get('https://celestrak.com/NORAD/elements/noaa.txt')
        f = open(file, 'wb')
        f.write(noaa.content)
        f.close()
    except:
        print('Could\'t download TLE sets. Will try again next time.')
        pass

    conn.close()

def read_in_config(file) -> tuple[imu.IMU, motor.Motor, motor.Motor]:
    """
    Reads in configuration file containing IMU registers and biases and motor pins and gear ratios
    :param file: the configuration file
    :return: an imu object and two motor objects
    """
    config = json.load(open(file, 'r'))
    bus = smbus.SMBus(1)
    devices_found = []
    for device in range(128):
        try:
            bus.read_byte(device)
            devices_found.append(hex(device))
        except OSError:
            pass

    accelerometers = config['accelerometer']
    magnetometers = config['magnetometer']
    device_address = [0, 0]
    accelerometer = None
    magnetometer = None
    for i in devices_found:
        str_add = str(int(i, 0))
        if str_add in accelerometers:
            device_address[0] = int(i, 0)
            accelerometer = accelerometers[str_add]
        elif str_add in magnetometers:
            device_address[1] = int(i, 0)
            magnetometer = magnetometers[str_add]

    acc_setup = accelerometer['setup']
    acc_registers = accelerometer['accreg']
    gyro_registers = accelerometer['gyroreg']
    gyro_scale = accelerometer['gyroscale']
    acc_bias = accelerometer['accbias']
    gyro_bias = accelerometer['gyrobias']

    # prepend device address to each register and join arrays
    setup = np.column_stack((np.full((len(acc_setup), 1), device_address[0]), acc_setup))

    if magnetometer is not None:
        mag_setup = magnetometer['setup']
        mag_registers = magnetometer['magreg']
        mag_bias_inv_a = magnetometer['magbiasa']
        mag_bias_b = magnetometer['magbiasb']
        mag_bias = (mag_bias_inv_a, mag_bias_b)
        setup = np.concatenate((setup, np.column_stack((np.full((len(mag_setup), 1), device_address[1]), mag_setup))),
                               axis=0)
    else:
        device_address = device_address[0]
        mag_registers = None
        mag_bias = None

    imu2 = imu.IMU(device_address, gyro_scale, acc_registers, gyro_registers, mag_registers, acc_bias, gyro_bias,
                   mag_bias, *setup)

    motors = config['motors']
    elevation_motor2 = motor.Motor(motors['elevation'], motors['elevationratio'])
    azimuth_motor2 = motor.Motor(motors['azimuth'], motors['azimuthratio'])

    return imu2, elevation_motor2, azimuth_motor2


def get_passes(satellite: sgp4lib.EarthSatellite, min_elevation: float) -> list:
    """
    Gets all upcoming passes for a satellite
    :param satellite: The satellite to get passes for
    :param min_elevation: The minimum elevation of the satellite
    :return: A list of satellite rise and set times
    """
    ts = load.timescale()
    utc = gps.gps.time
    now = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
    end = ts.utc(utc.year, utc.month, utc.day + 1, utc.hour, utc.minute, utc.second)

    t, events = satellite.find_events(gps.gps.location, now, end, min_elevation)
    passes = []
    for time, event in zip(t, events):
        pos = (satellite - gps.gps.location).at(time)
        (alt, _, _) = pos.altaz()
        time = dt.strptime(sftime.Time.utc_iso(time), "%Y-%m-%dT%H:%M:%S%z")
        # event 1 is the elevation peak
        # removes satellites with multiple elevation peaks
        if event == 1 and len(passes) > 1 and passes[-1][1] == 1:
            if passes[-1][0] < time:
                passes[-1] = (time, event, alt)
        else:
            passes.append((time, event, alt))
    passes = np.array_split(passes, int(len(passes) / 3))
    return passes


def get_next_pass(min_elevation: float) -> tuple[sgp4lib.EarthSatellite, list]:
    """
    Gets the next NOAA pass
    :param min_elevation: The minimum elevation
    :return: A tuple of the upcoming satellite and its rise and set times
    """
    all_sats = load.tle_file('noaa.txt')
    by_name = {sat.name: sat for sat in all_sats}
    noaa_15 = by_name['NOAA 15 [B]']
    noaa_18 = by_name['NOAA 18 [B]']
    noaa_19 = by_name['NOAA 19 [+]']
    sats = [noaa_15, noaa_18, noaa_19]

    # restrict to satellite elevation range
    if min_elevation < 0:
        min_elevation = 0
    elif min_elevation > 90:
        min_elevation = 90
    passes = {}
    for s in sats:
        passes[s] = get_passes(s, min_elevation)

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


def get_satellite_data(satellite: skyfield.sgp4lib.EarthSatellite) -> None:
    """
    Prints relevant live data about a satellite
    :param satellite: the satellite to print the data for
    :rtype: None
    :type satellite: skyfield.sgp4lib.EarthSatellite
    """

    ts = load.timescale()
    utc = gps.gps.time
    t = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
    geocentric = satellite.at(t)
    subpoint = wgs84.subpoint(geocentric)
    print('Latitude:', subpoint.latitude)
    print('Longitude:', subpoint.longitude)
    print('Height: {:.1f} km'.format(subpoint.antenna_elevation.km))
    pos = (satellite - gps.gps.location).at(t)
    alt, az, distance = pos.altaz()
    print('Altitude:', alt)
    print('Azimuth:', az)
    print('Distance: {:.1f} km'.format(distance.km))
    (_, _, sat_range, _, _, range_rate) = pos.frame_latlon_and_rates(gps.gps.location)
    print('Range: {:.1f} km'.format(sat_range.km))
    print('Range rate: {:.4f} km/s'.format(range_rate.km_per_s))


async def stream_decode_signal(s: sgp4lib.EarthSatellite, p: list) -> None:
    """
    Streams samples from RTL-SDR and decodes them
    :param s: The current satellite
    :param p: The rise and set times for the current satellite
    :return: None
    """

    filter_type = shared.filter_type
    print('Streaming started with filter ' + str(filter_type))

    if filter_type == 'grayscale':
        filter_type = None

    image = apt.APT(s.name, filter_type)
    finish_time = p[2][0]
    sample_rate = sdr.sample_rate
    # samples should be multiple of 512
    async for samples in sdr.stream(((sample_rate * 5) // 512) * 512):
        if gps.gps.time > finish_time or not shared.running.value:
            print('Streaming stopped at ' + finish_time)
            sdr.stop()

        image.decode_append(sample_rate, samples)
        image.export_image('media/image.png')



async def set_frequency(satellite: sgp4lib.EarthSatellite) -> None:
    """
    Sets the Doppler shifted frequency for a satellite
    :param satellite: The  currently tracked satellite
    :return: None
    """
    sdr.center_freq = int(get_frequency(satellite) + get_doppler_shift(satellite))


def signal_to_noise(s: np.ndarray, axis: int = 0) -> np.ndarray:
    """
    Calculates SNR for a signal
    :param s: the signal
    :param axis: the axis to calculate the standard deviation along
    :return: the SNR
    """
    mx = np.amax(s)
    s = np.divide(s, mx)
    s = np.square(s)
    s = np.asanyarray(s)
    m = s.mean(axis)
    sd = s.std(axis=axis)
    return np.where(sd == 0, 0, m / sd)


async def motor_controller(s: sgp4lib.EarthSatellite, p: list) -> None:
    """
    Control motor position to follow a satellite at a particular time
    :param s: the satellite to track
    :param p: the rise and set time for the satellite
    :return: None
    """
    print('motors starting')
    finish_time = p[2][0]
    # wait for accelerometer data
    await asyncio.sleep(1)
    while True:
        _, antenna_elevation, _ = imu.get_readings()
        antenna_azimuth = imu.get_bearing()
        if gps.gps.time > finish_time or not shared.running.value:
            print('motor stopped')
            print('resetting elevation motor')
            elevation_angle_delta = (90 - antenna_elevation)
            print('moving', 90 - antenna_elevation)
            await elevation_motor.rotate(elevation_angle_delta, 0)
            break
        ts = load.timescale()
        utc = gps.gps.time
        t = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
        pos = (s - gps.gps.location).at(t)
        sat_elevation, sat_azimuth, _ = pos.altaz()

        sat_elevation = abs(sat_elevation.degrees)
        sat_azimuth = sat_azimuth.degrees

        # convert satellite azimuth (0,+90,0,-90) to accelerometer (-90,0,+90,±180)
        if sat_elevation < 0:
            sat_elevation += 90
        else:
            sat_elevation = 90 - sat_elevation

        elevation_angle_delta = (antenna_elevation - sat_elevation)

        azimuth_angle_delta = (sat_azimuth - antenna_azimuth)

        # Move antenna clockwise or anticlockwise depending on which is closer
        if azimuth_angle_delta >= 180:
            azimuth_angle_delta -= 360
        elif azimuth_angle_delta < -180:
            azimuth_angle_delta += 360

        print('elevation moving ', abs(sat_elevation) - antenna_elevation, 'from', antenna_elevation)
        print('azimuth moving ', azimuth_angle_delta, 'from', antenna_azimuth)

        await elevation_motor.rotate(elevation_angle_delta, 0)

        await asyncio.gather(
            azimuth_motor.rotate(azimuth_angle_delta, 0),
            asyncio.sleep(30)
        )


async def doppler_task_controller(s: sgp4lib.EarthSatellite, p: list) -> None:
    """
    Adjusts frequency of RTL-SDR based on the Doppler shift of the satellite every 10 seconds
    :param s: The satellite to adjust the frequency for
    :param p: The rise and set times for the satellite
    :return: None
    """
    print('Doppler adjustment started')
    finish_time = p[2][0]
    while True:
        if gps.gps.time > finish_time or not shared.running.value:
            print('Doppler adjustment stopped')
            break
        await asyncio.gather(
            asyncio.sleep(10),
            set_frequency(s)
        )


async def wait_for_pass() -> tuple[sgp4lib.EarthSatellite, list]:
    print('waiting for pass')
    shared.waiting.value = True
    s, p = get_next_pass(MINIMUM_PASS_ANGLE)
    while p[0][0] > gps.gps.time:
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
            # set running value in case this method is triggered first
            shared.running.value = True
            # TODO revert this when not testing
            s, p = get_next_pass(MINIMUM_PASS_ANGLE)#await wait_for_pass()
            if shared.running.value:

                coro = [stream_decode_signal(s, p),
                        doppler_task_controller(s, p),
                        motor_controller(s, p),
                        imu.stream_readings(p)]
                await asyncio.gather(*coro, return_exceptions=True)
            print('done')
            if os.path.exists('media/image.png'):
                os.rename('media/image.png', 'media/previous/' + str(gps.gps.time) + '.png')


async def background_task_setup(app) -> None:
    asyncio.create_task(task_listener())
    asyncio.create_task(task_dispatcher())


if __name__ == '__main__':
    if not check_tle_epoch('noaa.txt'):
        update_tle('noaa.txt')
    imu, elevation_motor, azimuth_motor = read_in_config('deviceconfig.json')
    try:
        sdr = RtlSdr()
        sdr.sample_rate = 1.04e6
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
    app.add_routes([web.post('/running', emitter.load_start_web_interface)])
    aiohttp_jinja2.setup(app, loader=jinja2.FileSystemLoader(os.getcwd() + '/templates/'))
    web.run_app(app)
