import numpy as np
import scipy.signal as signal
from scipy.constants import physical_constants
from scipy.optimize import curve_fit
from PIL import Image

class APT(object):
    SYNC = 39
    SPACE = 47
    IMAGE = 909
    TELE = 45
    A_OFFSET = SYNC + SPACE + IMAGE + TELE
    B_OFFSET = A_OFFSET + SYNC + SPACE
    TELE_LENGTH = 128

    def __init__(self, sat_name, palette=None):
        self.sat_name = sat_name
        self.palette = palette
        self.image = []
        self.filtered_image = []
        self.tele_cal_index = 0
        self.telemetry = None
        self.thermal_cal_a = None
        self.thermal_cal_b = None
        self.channel = None
        self.cs = None
        self.black_body_temp = None

        if palette == 'precipitation':
            self.palette_data = np.array(Image.open('palettes/precipitation.png'))
        elif palette == 'thermal':
            self.palette_data = np.array(Image.open('palettes/thermal.png'))
        elif palette is not None:
            try:
                self.palette_data = np.array(Image.open('palettes/' + palette + '.png'))
            except FileNotFoundError:
                self.palette = None
        else:
            self.palette = None

    def decode_append(self, rate, data) -> None:
        """
        Decodes an array of complex values to an image. The image is returned as a matrix to enable chaining matrices
        together for simpler real time decoding
        :param rate: The sample rate of the data
        :param data: The data to be decoded
        :return: ndarray matrix
        """
        # APT broadcasts at 2 lines per second at 2080 words per line (4160 baud)
        # FM demodulation. Don't downsample to allow for super sampling the FM signal
        trunc = np.delete(data, 0)
        fm_demodulate = trunc.real * np.diff(data.imag) - trunc.imag * np.diff(data.real)
        del trunc
        # Signal is now double sideband, full carrier amplitude modulated 2400 Hz subcarrier
        # From APT manual: "Two channels of AVHRR data are time-division multiplexed into an output data stream that has
        # been pre-processed to achieve both bandwidth reduction and geometric correction."
        # "The averaging and geometric correction which produces 909 data samples per channel per line has an effective
        # bandwidth of 2080 Hz. To minimize distortion in the data, the signal is low-pass filtered prior to amplitude
        # modulation. The final output is AM modulated on a 2.4-KHz subcarrier and transferred directly to one of the
        # two APT transmitters."

        # Bandwidth of 2080 Hz means a AM radio spectrum bandwidth of 4160 Hz (2400±2080 Hz)
        # the lowest we can downsample to is 2x the bandwidth
        # sample rate should always be a multiple of 4160 Hz for n:1 downscaling
        sample_rate = 8320

        # Resample
        resample = signal.resample_poly(fm_demodulate, 500, int(rate / sample_rate * 500))
        # Delete full size array
        del fm_demodulate

        # Truncates end of signal so the total samples is an integer
        # so when we reshape to reduce to 1 sample per pixel the matrix is not irregular (ragged)
        limit = (len(resample) // sample_rate) * sample_rate
        trim = resample[:limit]

        # 2nd order butterworth highpass filter of 10 Hz (in fraction of Nyquist frequency)
        # The low frequencies are caused by DC components
        hp = signal.butter(2, 10 * 2 / sample_rate, 'hp', False, 'sos')
        highpass = signal.sosfilt(hp, trim)

        # Use hilbert transform to get analytic signal
        analytic = signal.hilbert(highpass)

        # The absolute value of the analytic signal is the amplitude demodulated signal
        # https://www.gaussianwaves.com/2017/04/extract-envelope-instantaneous-phase-frequency-hilbert-transform/
        # https://tomroelandts.com/articles/what-is-an-analytic-signal
        am_demodulate = np.abs(analytic)

        # Apply medium filter (window of 3) to reduce noise
        # might remove - adds time and doesn't change the image much (or sometimes makes it worse)
        medium = signal.medfilt(am_demodulate)

        # Take mean of every two elements, reduces array to 1 sample per pixel
        mean = np.mean(medium.reshape(len(medium) // 2, 2), axis=1)

        # Clip the 0.1th and 99.9th percentiles to increase contrast and scale to 8 bit
        (l, u) = np.percentile(mean, (0.1, 99.9))
        scale = 255 * (mean - l) / (u - l)
        scale[scale < 0] = 0
        scale[scale > 255] = 255

        # Round to integer
        decoded_signal = np.round(scale).astype(np.uint8)

        # find sync A frames and convert to 2d matrix
        # A Sync A frame is 39T, where T=1/4160s of the signal and corresponds to 1 pixel
        # When the signal is scaled to 8 bit unsigned integers, low is 11 and high is 244
        # It consists of a square wave of low amplitude for 4T, 7 cycles of high amplitude lasting 4T each (high for 2T,
        # low for 2T) then low amplitude for 7T (making the final low section 9T)
        # As the signal passes through an digital to analog converter before transmission, and then digitized again
        # on reception, the square wave appears as a (digital) full-wave rectified sine wave
        # Thus Sync A appears as zero amplitude for 2.5T, 7 rectified sine waves for 4T, then zero for 8.5T
        # As the .5 T values are zero amplitude, the reference frame can be clipped by 1T and these values rounded down
        # to ensure only the Sync A frame is captured
        # From NOAA-KLM User's Guide - Figure 4.2.3-3, page 4-53
        ideal_sync_a = [11] * 2 + [11, 116, 244, 116] * 7 + [11] * 8

        # Values are shifted back down to become symmetric waves (after we shifted them up by scaling to unsigned int)
        # This is so dot product can be used as a comparator
        symmetric_signal = [i - 128 for i in decoded_signal]
        ideal_sync_a = [i - 128 for i in ideal_sync_a]

        # List of sync a frame indexes and their similarity to the ideal Sync A frame
        sync_a = [(0, 0)]

        i = 0
        # Minus length of sync a to prevent dot product comparison of arrays of unequal size (less than sync a length)
        while i < len(decoded_signal) - len(ideal_sync_a):
            # Dot product of ideal sync a frame and the same length of signal
            # the more two waves are in sync, the higher their dot product will be
            similarity = np.dot(ideal_sync_a, symmetric_signal[i: i + len(ideal_sync_a)])

            # If the previous peak is far enough away to be a new peak, keep the previous peak and append current
            # a line will likely not be exactly 2080 pixels due to the Doppler effect
            if i - sync_a[-1][0] > 2000:
                sync_a.append((i, similarity))

            # If the current peak is on same line as the previous peak but with a higher correlation, replace it
            elif similarity > sync_a[-1][1]:
                sync_a[-1] = (i, similarity)

            # If current similarity is over 100,000, and position is > 100 pixels from start of line, skip to the lower
            # bound of the end value (2000)
            if sync_a[-1][1] > 100000 and i - sync_a[-1][0] > 100:
                i = sync_a[-1][0] + 2000

            i += 1

        image_matrix = []
        for i in range(len(sync_a) - 1):
            line = decoded_signal[sync_a[i][0]: sync_a[i][0] + 2080]
            if len(line) < 2080:
                line = np.pad(line, (0, 2080 - len(line)), 'constant')
            image_matrix.append(line)

        self.image.extend(image_matrix)

        image_matrix = np.array(image_matrix)
        if self.palette is not None:
            if self.palette == 'precipitation':
                self.apply_precipitation_filter(image_matrix)
            elif self.palette == 'thermal':
                self.apply_thermal_filter(image_matrix)
            elif self.palette is not None:
                self.apply_filter(image_matrix)


    def apply_filter(self, data) -> None:
        """
        Apply a false colour filter to the image
        :param data: the decoded image
        :return: None
        """
        # Extract only images from raw transmission
        channel_a = data[:, self.SYNC + self.SPACE:self.SYNC + self.SPACE + self.IMAGE]
        channel_b = data[:, self.B_OFFSET:self.B_OFFSET + self.IMAGE]
        image = np.zeros([len(channel_a), len(channel_a[0]), 3], dtype=np.uint8)

        # Use palette as look up table
        for y, x in np.ndindex(image.shape[:-1]):
            image[y][x] = self.palette_data[channel_b[y][x]][channel_a[y][x]]

        self.filtered_image.extend(image.tolist())


    def apply_precipitation_filter(self, data) -> None:
        """
        Apply precipitation filter
        :param data: the decoded data
        :return: None
        """
        data = np.repeat(data[:, :, np.newaxis], 3, axis=2)
        col_length, row_length, _ = data.shape
        # The max value is 255 and the length of the palette is 57 (when counting from zero)
        # The top values (197 to 255) will be transposed to the palette
        threshold = 255 - 57
        for y, x in np.ndindex(col_length, self.IMAGE):
            if data[y][x + self.B_OFFSET][0] >= threshold:
                val = data[y][x + self.B_OFFSET][0] - threshold
                data[y][x + self.B_OFFSET] = self.palette_data[val][0]
                data[y][x] = self.palette_data[val][0]

        self.filtered_image.extend(data.tolist())

    def apply_thermal_filter(self, data) -> None:
        """
        Calculates the temperature in Kelvin of each pixel based on NOAA calibration values
        Formulae from section 7.1.2.4 NOAA/KLM User's Guide, page 7-7
        :param data: The decoded data
        :return: None
        """

        # If not enough telemetry data downloaded
        if len(self.image) < self.TELE_LENGTH:
            return

        lines_since_last_cal = len(self.image) - self.tele_cal_index

        if lines_since_last_cal > self.TELE_LENGTH:
            try:
                self.calibrate_with_tele_frame(data)
            except ValueError:
                return

        if self.channel < 4:
            # Not an infrared image
            return

        # If this is the first run since start
        if len(self.filtered_image) == 0:
            data = self.image

        # clip to infrared image
        data = data[:, self.B_OFFSET-3:self.B_OFFSET + self.IMAGE-5]

        channel = self.channel - 4

        cal = self.get_thermal_calibration()

        c1 = physical_constants["first radiation constant for spectral radiance"][0] * 1e11
        c2 = physical_constants["second radiation constant"][0] * 100

        # Black body radiance temp
        c = cal[1][channel][0]
        nbb = c1 * c ** 3 / (np.expm1(c2 * c / self.black_body_temp))

        # Black body
        cs = self.cs * 4
        cb = self.telemetry[14] * 4

        # Store heat transfer coefficients
        ns = cal[2][channel][0]
        b_all = cal[2][channel][1]
        b0 = b_all[0]
        b1 = b_all[1]
        b2 = b_all[2]
        vc = cal[1][channel][0]
        a = cal[1][channel][1]
        b = cal[1][channel][2]

        # 4 element array as palette has transparency (RGBA)
        image = np.zeros([len(data), len(data[0]), 4], dtype=np.uint8)
        palette = self.palette_data

        for y, x in np.ndindex(data.shape):
            ce = self.fit_bias(data[y][x], self.thermal_cal_a, self.thermal_cal_b)
            nl = ns + (nbb - ns) * (cs - ce * 4) / (cs - cb)
            nc = b0 + b1 * nl + b2 * nl * nl

            ne = nl + nc

            t = c2 * vc / np.log(c1 * vc * vc * vc / ne + 1)
            t = t - a / b
            t -= 273.15
            t = (t + 100) / 160 * 255
            image[y][x] = palette[np.uint8(t)][0]

        self.filtered_image.extend(image.tolist())


    def export_image(self, filename) -> None:
        """
        Exports the matrix image to a file
        :param filename: the filename to save the image as
        :return:
        """
        if self.palette is None or len(self.filtered_image) == 0:
            Image.fromarray(np.array(self.image, dtype=np.uint8)).save(filename)
        else:
            Image.fromarray(np.array(self.filtered_image, dtype=np.uint8)).save(filename)


    def fit_bias(self, data, a, b):
        """
        Model function for least squares fitting
        :param data: the value to be adjusted
        :param a: the first fit parameter
        :param b: the second fit parameter
        :return: adjusted data
        """
        return (a * data) + b


    def calibrate_with_tele_frame(self, data) -> None:
        """
        Finds the best match telemetry frame in the image and finds calibration values, space count and
        black body temperature using it
        :param data: the APT transmission
        :return: None
        """
        if len(data) < self.TELE_LENGTH:
            raise ValueError

        # Extract Telemetry frame and take horizontal mean
        tele_b = data[:, self.B_OFFSET + self.IMAGE: self.B_OFFSET + self.IMAGE + 40]
        tele_b = np.mean(tele_b, axis=1)

        space_b = data[:, self.B_OFFSET - self.SPACE: self.B_OFFSET]
        space_b = np.mean(space_b, axis=1)

        max_sim = 0
        start = 0
        # Identify channel 9/10 of any telemetry frame by finding the largest white/black difference
        for i in range(4, len(tele_b)-4):
            similarity = sum(tele_b[i-4:i-1]) - sum(tele_b[i:i+4])
            if similarity > max_sim:
                start = i
                max_sim = similarity

        # If a complete frame isn't available, raise exception
        if tele_b[start] + self.TELE_LENGTH > len(tele_b):
            raise ValueError

        # Clip to start/end of telemetry frames, take mean vertically, then split into individual frames
        first = (start + (self.TELE_LENGTH // 2)) % self.TELE_LENGTH
        tele_b = tele_b[first:-1 * ((len(tele_b) - first) % self.TELE_LENGTH)]
        tele_b = np.mean(tele_b.reshape(-1, 8)[:, 1:-1], axis=1)
        tele_b = tele_b.reshape(-1, 16)
        space_b = space_b[first:-1 * ((len(space_b) - first) % self.TELE_LENGTH)]
        space_b = space_b.reshape(-1, self.TELE_LENGTH)

        # Find best match telemetry frame
        max_sim = 0
        ideal_tele = [31.07, 63.02, 94.96, 126.9, 158.86, 191.1, 228.62, 255.0, 0.0]
        match_index = -1
        tele_b_match = []
        space_b_match = []

        for i in range(len(tele_b)):
            similarity = np.dot(tele_b[i][:9], ideal_tele)
            if similarity > max_sim:
                max_sim = similarity
                match_index = i
                tele_b_match = tele_b[i]
                space_b_match = space_b[i]

        self.tele_cal_index = match_index * self.TELE_LENGTH + first

        # Nonlinear regression to match telemetry frame to ideal
        (a, b), _ = curve_fit(self.fit_bias, tele_b_match[:9], ideal_tele)
        self.thermal_cal_a = a
        self.thermal_cal_b = b
        tele_b_match = self.fit_bias(tele_b_match, a, b)

        self.telemetry = tele_b_match

        # Find channel of image
        max_sim = 255
        channel = -1
        for i in range(6):
            similarity = abs(tele_b_match[i] - tele_b_match[15])
            if similarity < max_sim:
                max_sim = similarity
                channel = i + 1

        self.channel = channel

        # Average space count (p 7-6)
        cs = 0
        count = 0
        for i in range(len(space_b_match)):
            if space_b_match[i] > 50:
                cs += space_b_match[i]
                count += 1

        self.cs = self.fit_bias((cs / count), a, b)

        # Calculate blackbody temperature - page 7-7, section 7.1.2.4 NOAA/KLM User's Guide
        temp = [0, 0, 0, 0]
        cal = self.get_thermal_calibration()
        for i in range(4):
            c = self.telemetry[i + 9] * 4
            d0 = cal[0][i][0]
            d1 = cal[0][i][1]
            d2 = cal[0][i][2]
            temp[i] = d0
            temp[i] += d1 * c
            c *= c
            temp[i] += d2 * c

        black_body = np.mean(temp)
        black_body = cal[1][channel - 4][1] + cal[1][channel - 4][2] * black_body

        self.black_body_temp = black_body


    def get_thermal_calibration(self) -> list:
        """
        Get calibration values for AVHRR. From NOAA KLM User's Guide, page numbers in comments
        :return: Calibration values for the current satellite
        """
        if self.sat_name == 'NOAA 15 [B]':
                # PRT coefficient d0, d1, d2 - Table D.1-8, pD-11 or 956
            return [[[276.60157, 0.051045, 1.36328E-06],
                    [276.62531, 0.050909, 1.47266E-06],
                    [276.67413, 0.050907, 1.47656E-06],
                    [276.59258, 0.050966, 1.47656E-06]],
                # Channel radiance coefficient vc, A, B - Table D1-11, pD-18 or 963
                    [[925.4075, 0.337810, 0.998719], # Channel 4
                    [839.8979, 0.304558, 0.999024], # Channel 5
                    [2695.9743, 1.621256, 0.998015]], # Channel 3B
                # Nonlinear radiance correction Ns, b0, b1, b2 - Table D.1-14, pD-37 or 982
                    [[-4.50, [4.76, -0.0932, 0.0004524]], # Channel 4
                    [-3.61, [3.83, -0.0659, 0.0002811]], # Channel 5
                    [0.0, [0.0, 0.0, 0.0]]]] # Channel 3B
        elif self.sat_name == 'NOAA 18 [B]':
                # PRT coefficient d0, d1, d2
            return [[[276.601, 0.05090, 1.657e-06],
                    [276.683, 0.05101, 1.482e-06],
                    [276.565, 0.05117, 1.313e-06],
                    [276.615, 0.05103, 1.484e-06]],
                 # Channel radiance coefficient vc, A, B
                    [[928.1460, 0.436645, 0.998607], # Channel 4
                    [833.2532, 0.253179, 0.999057], # Channel 5
                    [2659.7952, 1.698704, 0.996960]], # Channel 3B
                # Nonlinear radiance correction Ns, b0, b1, b2
                    [[-5.53, [5.82, -0.11069, 0.00052337]], # Channel 4
                    [-2.22, [2.67, -0.04360, 0.00017715]], # Channel 5
                    [0.0, [0.0, 0.0, 0.0]]]] # Channel 3B
        elif self.sat_name == 'NOAA 19 [+]':
                # PRT coefficient d0, d1, d2
            return [[[276.6067, 0.051111, 1.405783E-06],
                    [276.6119, 0.051090, 1.496037E-06],
                    [276.6311, 0.051033, 1.496990E-06],
                    [276.6268, 0.051058, 1.493110E-06]],
                 # Channel radiance coefficient vc, A, B
                    [[928.9, 0.53959, 0.998534], # Channel 4
                    [831.9, 0.36064, 0.998913], # Channel 5
                    [2670.0, 1.67396, 0.997364]], # Channel 3B
                 # Nonlinear radiance correction Ns, b0, b1, b2
                    [[-5.49, [5.70, -0.11187, 0.00054668]], # Channel 4
                    [-3.39, [3.58, -0.05991, 0.00024985]], # Channel 5
                    [0.0, [0.0, 0.0, 0.0]]]] # Channel 3B




