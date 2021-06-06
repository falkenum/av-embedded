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

(fs, x) = UF.wavread('../sounds/flute-A4.wav')
p = serial.Serial('/dev/ttyACM1', 115200)
M = 1024
# p.write(int(255).to_bytes(1, 'big', True))

# for i in range(M):
#     data = struct.pack('<f', x[:M][i])
#     p.write(data)

X2 = np.zeros(M//2)

max = 0.0
min = 0.0

data = p.read(4*M//2)

for i in range(M//2):
    X2[i] = struct.unpack('<f', data[i*4:(i+1)*4])[0]
    if X2[i] > max:
        max = X2[i]
    if X2[i] < min:
        min = X2[i]

print(max)
print(min)

w = np.zeros(M)

for i in range(M):
    w[i] = 25/46+(1-25/46)*np.cos(2*np.pi*i/M)
X1 = 20*np.log10(abs(fft(x[:M]*w, M)))
plt.plot(X1)
plt.plot(X2)
plt.show()