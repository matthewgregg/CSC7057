import numpy as np
import scipy.signal as signal


def decode(rate, data):
    """
    Decodes the stream of complex values to an image. The image is returned as a matrix to enable chaining matrices
    together for simpler real time decoding
    """
    # APT broadcasts at 2 lines per second at 2080 words per line (4160 baud)
    # apt_symbol_rate = 4160

    # FM demodulation. Don't downsample to allow for super sampling the FM signal
    trunc = np.delete(data, 0)
    fm_demodulate = trunc.real * np.diff(data.imag) - trunc.imag * np.diff(data.real)
    del trunc

    # Signal is now double sideband, full carrier amplitude modulated 2400 Hz subcarrier
    # "Two channels of AVHRR data are time-division multiplexed into an output data stream that has
    # been pre-processed to achieve both bandwidth reduction and geometric correction."
    # "The averaging and geometric correction which produces 909 data samples per channel per line has an effective
    # bandwidth of 2080 Hz. To minimize distortion in the data, the signal is low-pass filtered prior to amplitude
    # modulation. The final output is AM modulated on a 2.4-KHz subcarrier and transferred directly to one of the
    # two APT transmitters."

    # bandwidth of 2080 Hz means a AM radio spectrum bandwidth of 4160 Hz (2400±2080 Hz)
    # the lowest we can downsample to is 2x the bandwidth
    # sample rate should always be a multiple of 4160 Hz for n:1 downscaling
    sample_rate = 8320

    # resample
    resample = signal.resample_poly(fm_demodulate, 500, int(rate / sample_rate * 500))

    # truncates end of signal so the total samples is an integer
    # so when we reshape to reduce to 1 sample per pixel the matrix is not irregular (ragged)
    limit = (len(resample) // sample_rate) * sample_rate
    trim = resample[:limit]

    # 2nd order butterworth highpass filter of 10 Hz (in fraction of Nyquist frequency)
    # the low frequencies are caused by DC components
    hp = signal.butter(2, 10 * 2 / sample_rate, 'hp', False, 'sos')
    highpass = signal.sosfilt(hp, trim)

    # use hilbert transform to get analytic signal
    analytic = signal.hilbert(highpass)

    # the absolute value of the analytic signal is the amplitude demodulated signal
    # https://www.gaussianwaves.com/2017/04/extract-envelope-instantaneous-phase-frequency-hilbert-transform/
    # https://tomroelandts.com/articles/what-is-an-analytic-signal
    am_demodulate = np.abs(analytic)

    # apply medium filter (window of 3) to reduce noise
    # might remove - adds time and doesn't change the image much (or sometimes makes it worse)
    medium = signal.medfilt(am_demodulate)

    # take mean of every two elements, reduces array to 1 sample per pixel
    mean = np.mean(medium.reshape(len(medium) // 2, 2), axis=1)

    # clip the 0.1th and 99.9th percentiles to increase contrast and scale to 8 bit
    (l, u) = np.percentile(mean, (0.1, 99.9))
    scale = 255 * (mean - l) / (u - l)
    scale[scale < 0] = 0
    scale[scale > 255] = 255

    # round to integer
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

    # list of sync a frame indexes and their similarity to the ideal Sync A frame
    sync_a = [(0, 0)]

    # values are shifted back down to become symmetric waves (after we shifted them up by scaling to unsigned int)
    # This is so dot product can be used as a comparator
    symmetric_signal = [i - 128 for i in decoded_signal]
    ideal_sync_a = [i - 128 for i in ideal_sync_a]

    i = 0
    # minus length of sync a to prevent dot product comparison of arrays of unequal size (less than sync a length)
    while i < len(decoded_signal) - len(ideal_sync_a):
        # dot product of ideal sync a frame and the same length of signal
        # the more two waves are in sync, the higher their dot product will be
        similarity = np.dot(ideal_sync_a, symmetric_signal[i: i + len(ideal_sync_a)])

        # if the previous peak is far enough away to be a new peak, keep the previous peak and append current
        # the minimum value is 2000 as the line may not be exactly 2080 pixels due to the Doppler effect
        if i - sync_a[-1][0] > 2000:
            sync_a.append((i, similarity))

        # if the current peak is on same line as the previous peak but with a higher correlation, replace it
        elif similarity > sync_a[-1][1]:
            sync_a[-1] = (i, similarity)

        # if current similarity is over 100,000, and position is > 100 pixels from start of line, skip to the lower
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

    return image_matrix

