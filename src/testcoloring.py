from scipy.io import wavfile
import apt
from PIL import Image
import numpy as np

rate, data = wavfile.read('test/testsignal.wav')
image_raw = Image.open('image.png').convert('L')
real = data[:,0]
imag = data[:,1]
data = real + imag * 1j

image = apt.APT('NOAA 19 [+]', 'precipitation')
image.decode_append(rate, data)
# image.apply_precipitation_filter(data)
# image.apply_thermal_filter(data)

image.export_image('colourtest.png')