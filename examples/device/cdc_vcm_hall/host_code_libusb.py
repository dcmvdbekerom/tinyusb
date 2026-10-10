# -*- coding: utf-8 -*-
"""
Created on Sun Sep 27 12:22:36 2026

@author: dcmvd
"""

import os
#os.environ['LIBUSB_DEBUG'] = '4'   # drop this now that things work, if you want quieter output

import libusb_package
import usb.core
import usb.util
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter


    
chunk_size = 512
N_chunks = 1024
buf_size = chunk_size * N_chunks
read_buf = np.zeros(buf_size, dtype=np.uint8)

fs = 6e7 / 15 / 16 #Samples/s


#%% USB acquisition

backend = libusb_package.get_libusb1_backend()
dev = usb.core.find(idVendor=0xCAFE, idProduct=0x133F, 
                    backend=backend,
                    )
if dev is None:
    raise RuntimeError("Device not found")

intf = dev[0][(2, 0)]
ep_out = intf[0]
ep_in  = intf[1]

try:
    
    send_arr = np.array([0x55AA55AA], dtype="<u4")  # START
    print("START acquisition")
    ep_out.write(send_arr.tobytes())

    print("Acquiring...")

    
    t0 = perf_counter()
    read_bytes = 0

    ydata8 = np.zeros(chunk_size, dtype=np.uint8)
    ydata = ydata8.view(np.uint16)
    # p0, = plt.plot(ydata)
    # fig = plt.gcf()
    # plt.ylim(0,10000)
    
    while read_bytes < buf_size:
        # print(read_bytes)
        raw = ep_in.read(chunk_size, timeout=1000)
        # print(len(raw))
        # ydata8[0:len(raw)] = raw
        # p0.set_ydata(ydata)
        
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        
        read_buf[read_bytes:read_bytes+len(raw)] = raw
        read_bytes += len(raw)
    
    
    
    t1 = perf_counter()
    send_arr = np.array([0x00FF00FF], dtype="<u4")  # STOP
    print("STOP acquisition")
    ep_out.write(send_arr.tobytes())
    
    delta_t = t1 - t0 #s
    UINTSIZE = 2
    t_expd = buf_size / UINTSIZE / fs #s
    t_ratio = 100. * (delta_t - t_expd) / t_expd
    
    print(f'Time elapsed:  {delta_t:.2f}s (expected {t_expd:.2f}s / {t_ratio:+01.1f}%)')
    

finally:
    usb.util.dispose_resources(dev)
    print("Connection closed.")


    
#%% Plot data

# diff = np.ediff1d(read_data, to_begin=1)
# plt.plot(diff, '-') 
# print(np.arange(len(read_data))[diff > 2])


read_data = read_buf.view(np.uint16)
y_arr = read_data.astype(np.float64) / 4096. * 3.3
dt = 1/fs #s
t_arr = np.arange(len(read_data))*dt
# plt.plot(t_arr, y_arr)

from scipy import fft

f_arr = fft.rfftfreq(len(t_arr), d=dt) *1e-3 #kHz
Y_arr = fft.rfft(y_arr)

plt.plot(f_arr, Y_arr)
plt.yscale('log')




        