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

# for i in range(M):
#     data = struct.pack('<f', x[:M][i])
#     p.write(data)

X2 = np.zeros(M//2)

for i in range(M//2):
    data = p.read(4)
    X2[i] = struct.unpack('<f', data)[0]


w = np.zeros(M)

for i in range(M):
    w[i] = 25/46+(1-25/46)*np.cos(2*np.pi*i/M)
# xw = x[:M]*w
X1 = 20*np.log10(abs(fft(x[:M]*w, M)))
plt.plot(X1)
plt.plot(X2)
plt.show()