import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt

# Reads the sample rate and data from the .wav file.
sample_rate, data = wavfile.read('C:/Users/sugar/Downloads/EE14/EE14_Final_Project/src/Note_C.wav')

# Checks to see if the audio has two channels (stereo), if it does, it converts 
# it to a single channel audio (mono).
if data.ndim == 2:
    data = data.mean(axis=1).astype(data.dtype)

data = data.astype(np.float32)

# Since we want to use the 12-bit resolution on the DAC, we will convert
# the data into 12 bit integers.
converted_data = ((data - data.min()) / (data.max() - data.min()) * 4095).astype(np.uint16)

# for i in range (20000, 56000, 100):
#     print(f'{converted_data[i]},')

# print(f"Data min: {data.min()}")
# print(f"Data max: {data.max()}")
# print(f"Data mean: {data.mean()}")
# print(f"Data standard deviation: {data.std()}")

plt.plot(converted_data)
plt.title('Converted Audio Waveform')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude (12-bit)')
plt.show()


