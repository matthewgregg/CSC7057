import aiohttp.web_app
import jinja2
import aiohttp_jinja2
import skyfield
from aiohttp import web
import asyncio
import os.path
from datetime import datetime as dt, timedelta, timezone
import numpy as np
from scipy import constants
from scipy.io import wavfile
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
    running_value = Property()

    async def load_stop_web_interface(self, request):
        """Loads the stopped web interface

        :param request: The HTTP request
        :return: The HTML template to be displayed
        """
        data = {
            'running': 0,
        }
        self.running_value = False
        return aiohttp_jinja2.render_template('index.html', request, data)

    async def load_start_web_interface(self, request):
        """Loads the started web interface

        :param request: The HTTP request
        :return: The HTML template to be displayed
        """
        post = await request.post()

        s, p = get_next_pass(MINIMUM_PASS_ANGLE)
        sat_name = s.name
        rise_time = dt.isoformat(p[0][0])
        set_time = dt.isoformat(p[2][0])

        data = {'running': 1,
                'satellite': sat_name,
                'rise': rise_time,
                'set': set_time,
                'sdrerror': sdr_error,
                'redirect': False}

        try:
            shared.filter_type = post['filter-type']
        except KeyError:
            self.running_value = False
            data['redirect'] = True

        if shared.running.value and not shared.waiting.value:
            self.running_value = False
            data['redirect'] = True

        if sdr_error == 0 and data['redirect'] is not True:
            self.running_value = True

        return aiohttp_jinja2.render_template('index.html', request, data)


def get_frequency(satellite: sgp4lib.EarthSatellite) -> int:
    """Get the frequency for a satellite

    Get the nominal APT frequency of a NOAA satellite
    :param satellite: The satellite to get the nominal frequency of
    :return: The nominal APT frequency
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
    """Calculates Doppler shift

    Calculates the Doppler shift based on the range rate of a satellite
    :param satellite: The satellite to calculate the Doppler shift for
    :return: The Doppler shifted frequency
    """
    t = load.timescale().now()
    pos = (satellite - gps.gps.location).at(t)
    (_, _, _, _, _, range_rate) = pos.frame_latlon_and_rates(gps.gps.location)
    return range_rate.m_per_s / constants.c * get_frequency(satellite)


def check_tle_epoch(filename: str) -> bool:
    """Checks if TLE is outdated

    Checks whether the Two Line Element Set is outdated by parsing its date
    :return: False if outdated
    """
    if os.path.exists(filename):
        with open(filename, "r+") as r:
            line = r.readlines()
            tle_year = int(line[1][18:20])
            # TLE year is 2 digits from 1957-2056
            tle_year += (1900 if tle_year >= 57 <= 99 else 2000)
            tle_day = float(line[1][20:32])
            tle_date = (dt(tle_year, 1, 1) + timedelta(tle_day - 1)).replace(tzinfo=timezone.utc)
            tle_outdated = (gps.gps.time - tle_date).total_seconds() > 7 * 24 * 3600

    if not os.path.exists(filename) or tle_outdated:
        return False

    return True


def update_tle(filename: str) -> None:
    """Updates TLE file

    :param filename: the filename of the TLE file
    :return: None
    """
    import requests
    try:
        noaa = requests.get('https://celestrak.com/NORAD/elements/noaa.txt')
        f = open(filename, 'wb')
        f.write(noaa.content)
        f.close()
    except requests.ConnectionError:
        print('Could\'t download TLE sets. Will try again next time.')
        pass


def read_in_config(filename: str) -> tuple[imu.IMU, motor.Motor, motor.Motor]:
    """Read in IMU and motor configuration

    Reads in configuration file containing IMU registers and biases and motor pins and gear ratios
    :param filename: The name of the configuration file
    :return: An imu object and two motor objects
    """
    config = json.load(open(filename, 'r'))
    bus = smbus.SMBus(1)
    devices_found = []
    # Get devices on i2c
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

    # Match i2c devices to devices in configuration file
    for i in devices_found:
        str_add = str(int(i, 0))
        if str_add in accelerometers:
            device_address[0] = int(i, 0)
            accelerometer = accelerometers[str_add]
        elif str_add in magnetometers:
            device_address[1] = int(i, 0)
            magnetometer = magnetometers[str_add]

    if accelerometer is None:
        raise ValueError("Accelerometer not found")

    acc_setup = accelerometer['setup']
    acc_registers = accelerometer['accreg']
    gyro_registers = accelerometer['gyroreg']
    gyro_scale = accelerometer['gyroscale']
    acc_bias = accelerometer['accbias']
    gyro_bias = accelerometer['gyrobias']

    # Prepend device address to each register
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
    elevation_motor2 = motor.Motor(motors['elevation'], motors['elevationratio'], motors['elevationmax'])
    azimuth_motor2 = motor.Motor(motors['azimuth'], motors['azimuthratio'], motors['azimuthmax'])

    return imu2, elevation_motor2, azimuth_motor2


def get_passes(satellite: sgp4lib.EarthSatellite, min_elevation: float) -> list:
    """Gets all upcoming passes for a satellite

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
        (elevation, _, _) = pos.altaz()
        time = dt.strptime(sftime.Time.utc_iso(time), "%Y-%m-%dT%H:%M:%S%z")
        # Event 1 is the elevation peak
        # Removes satellites with multiple elevation peaks
        if event == 1 and len(passes) > 1 and passes[-1][1] == 1:
            if passes[-1][0] < time:
                passes[-1] = (time, event, elevation)
        else:
            passes.append((time, event, elevation))
    passes = np.array_split(passes, int(len(passes) / 3))
    return passes


def get_next_pass(min_elevation: float) -> tuple[sgp4lib.EarthSatellite, list]:
    """Get the next NOAA pass

    :param min_elevation: The minimum elevation
    :return: A tuple of the upcoming satellite and its rise and set times
    """
    all_sats = load.tle_file('noaa.txt')
    by_name = {sat.name: sat for sat in all_sats}
    noaa_15 = by_name['NOAA 15 [B]']
    noaa_18 = by_name['NOAA 18 [B]']
    noaa_19 = by_name['NOAA 19 [+]']
    sats = [noaa_15, noaa_18, noaa_19]

    # Restrict to satellite elevation range
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
        elevation = 2

        if next_pass[rise][time] < closest[1][rise][time]:
            if next_pass[rise][time] < closest[1][setting][time]:
                closest = (s, next_pass)
            # If set time overlaps with rise time of next, take the one with the highest peak elevation
            elif closest[1][peak][elevation] > next_pass[peak][elevation]:
                closest = (s, next_pass)
    return closest


def get_satellite_data(satellite: skyfield.sgp4lib.EarthSatellite) -> None:
    """Prints relevant live data about a satellite

    :param satellite: the satellite to print the data for
    :return: None
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
    elevation, azimuth, distance = pos.altaz()
    print('Elevation/Altitude:', elevation)
    print('Azimuth:', azimuth)
    print('Distance: {:.1f} km'.format(distance.km))
    (_, _, sat_range, _, _, range_rate) = pos.frame_latlon_and_rates(gps.gps.location)
    print('Range: {:.1f} km'.format(sat_range.km))
    print('Range rate: {:.4f} km/s'.format(range_rate.km_per_s))


async def stream_decode_signal(satellite: sgp4lib.EarthSatellite, passes: list) -> None:
    """Streams samples from RTL-SDR and passes them to APT object

    :param satellite: The current satellite
    :param passes: The rise and set times for the current satellite
    :return: None
    """
    filter_type = shared.filter_type
    print('Streaming started with filter ' + str(filter_type))

    if filter_type == 'grayscale':
        filter_type = None

    image = apt.APT(satellite.name, filter_type)
    finish_time = passes[2][0]
    sample_rate = sdr.sample_rate
    time = 5
    # Samples should be multiple of 512
    async for samples in sdr.stream(((sample_rate * time) // 512) * 512):
        if gps.gps.time > finish_time or not shared.running.value:
            print('Streaming stopped at ' + finish_time)
            sdr.stop()

        image.decode_append(sample_rate, samples)
        image.export_image('media/image.png')


async def set_frequency(satellite: sgp4lib.EarthSatellite) -> None:
    """Set the Doppler shifted frequency for a satellite

    :param satellite: The  currently tracked satellite
    :return: None
    """
    frequency = int(get_frequency(satellite) + get_doppler_shift(satellite))
    print('frequency set to ' + str(frequency))
    sdr.center_freq = frequency


def signal_to_noise(signal: np.ndarray, axis: int = 0) -> np.ndarray:
    """Calculates SNR for a signal

    :param signal: the signal
    :param axis: the axis to calculate the standard deviation along
    :return: the SNR
    """
    mx = np.amax(signal)
    signal = np.divide(signal, mx)
    signal = np.square(signal)
    signal = np.asanyarray(signal)
    m = signal.mean(axis)
    sd = signal.std(axis=axis)
    return np.where(sd == 0, 0, m / sd)


async def motor_controller(satellite: sgp4lib.EarthSatellite, passes: list) -> None:
    """Track a satellite by rotating motors

    Adjust elevation and azimuth motor position to follow a satellite
    :param satellite: the satellite to track
    :param passes: the rise and set time for the satellite
    :return: None
    """
    print('Motors starting')
    finish_time = passes[2][0]
    # Wait for accelerometer data
    await asyncio.sleep(1)
    while True:
        _, antenna_elevation, _ = imu.readings
        antenna_azimuth = imu.bearing
        if gps.gps.time > finish_time or not shared.running.value:
            print('Motor stopped')
            print('Resetting elevation motor')
            print('Moving', antenna_elevation)
            await elevation_motor.rotate(antenna_elevation, 0)
            break
        ts = load.timescale()
        utc = gps.gps.time
        t = ts.utc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second)
        pos = (satellite - gps.gps.location).at(t)
        sat_elevation, sat_azimuth, _ = pos.altaz()

        sat_elevation = abs(sat_elevation.degrees)
        sat_azimuth = sat_azimuth.degrees

        # Convert satellite azimuth (0,+90,0,-90) to accelerometer (-90,0,+90,±180)
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


async def doppler_task_controller(satellite: sgp4lib.EarthSatellite, passes: list) -> None:
    """Adjusts SDR frequency every 10 seconds

    Adjusts frequency of RTL-SDR based on the Doppler shift of the satellite every 10 seconds
    :param satellite: The satellite to adjust the frequency for
    :param passes: The rise and set times for the satellite
    :return: None
    """
    print('Doppler adjustment started')
    finish_time = passes[2][0]
    while True:
        if gps.gps.time > finish_time or not shared.running.value:
            print('Doppler adjustment stopped')
            break
        await asyncio.gather(
            asyncio.sleep(10),
            set_frequency(satellite)
        )


async def wait_for_pass() -> tuple[sgp4lib.EarthSatellite, list]:
    """Finds the next satellite and waits until it's rise time

    :return: The next satellite and its rise and set times
    """
    print('Waiting for pass')
    shared.waiting.value = True
    s, p = get_next_pass(MINIMUM_PASS_ANGLE)
    while p[0][0] > gps.gps.time:
        await asyncio.sleep(0.5)
        if not shared.running.value:
            break
    shared.waiting.value = False
    return s, p


async def task_listener() -> None:
    """Listens for Property state change

    Listens for the state of the Property to change in WebInterface() and updates the value in the class Running()
    :return: None
    """
    while True:
        (_, shared.running.value), _ = await running_event
        print('Running value changed to', shared.running.value)


async def task_dispatcher() -> None:
    """Launches coroutines when tracking begins

    :return: None
    """
    while True:
        (_, running_val), _ = await running_event
        if running_val:
            # Set running value in case this method is triggered first
            shared.running.value = True
            sat, passes = get_next_pass(MINIMUM_PASS_ANGLE)
            # sat, passes = await wait_for_pass()
            if shared.running.value:
                coroutines = [stream_decode_signal(sat, passes),
                              doppler_task_controller(sat, passes),
                              motor_controller(sat, passes),
                              imu.stream_readings(passes)]
                await asyncio.gather(*coroutines, return_exceptions=True)
            print('Done')
            if os.path.exists('media/image.png'):
                os.rename('media/image.png', 'media/previous/' + str(gps.gps.time) + '.png')


async def background_task_setup(app: aiohttp.web_app.Application) -> None:
    """Run task_listener() and task_dispatcher()

    Runs task_listener() and task_dispatcher() asynchronously. As task_listener() controls the state of Running,
    the value can be updated independently of task_dispatcher(), allowing it to be cancelled
    :param app: The asyncio web server application. Required to attach to asyncio background tasks
    :return: None
    """
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
    running_event = emitter.get_dispatcher_event('running_value')
    app = web.Application()
    app.on_startup.append(background_task_setup)
    app.add_routes([web.get('/', emitter.load_stop_web_interface)])
    app.add_routes([web.get('/running', emitter.load_start_web_interface)])
    app.add_routes([web.static('/media', os.getcwd() + '/media/')])
    app.add_routes([web.post('/running', emitter.load_start_web_interface)])
    aiohttp_jinja2.setup(app, loader=jinja2.FileSystemLoader(os.getcwd() + '/templates/'))
    web.run_app(app)
