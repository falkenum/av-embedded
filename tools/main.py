import numpy as np
import time, os, sys
import serial, struct

# sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../software/models/'))

# import stft as STFT
import utilFunctions as UF
import matplotlib.pyplot as plt
from scipy.signal import hamming
from scipy.fftpack import fft
import math

(fs, x) = UF.wavread('../sounds/piano.wav')
p = serial.Serial('/dev/ttyACM0', 115200)
M = 1024
# p.write(int(255).to_bytes(1, 'big', True))

xint = np.multiply(x[:M], (1 << 15)) + (1 << 15)
print(x[0])
for i in range(len(xint)):
    sample = int(xint[i])
    data = sample.to_bytes(2, 'big')
    p.write(data)
    p.write(b',')

X2 = np.zeros(M//2)

for i in range(M//2):
    data = p.read(4)
    comma = p.read()
    # unpacked = struct.unpack('<f', data)
    X2[i] = struct.unpack('<f', data)[0]


# w = np.hamming(M)
# xw = x[:M]*w
X1 = 20*np.log10(abs(fft(x, M)))
plt.plot(X1)
plt.plot(X2)
plt.show()