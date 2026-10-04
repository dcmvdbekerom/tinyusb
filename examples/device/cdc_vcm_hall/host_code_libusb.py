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

fs = 2e5 #Samples/s


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
    while read_bytes < buf_size:
        # print(read_bytes)
        raw = ep_in.read(chunk_size, timeout=1000)
        read_buf[read_bytes:read_bytes+len(raw)] = raw
        read_bytes += len(raw)
    
    t1 = perf_counter()


    send_arr = np.array([0x00FF00FF], dtype="<u4")  # STOP
    print("STOP acquisition")
    ep_out.write(send_arr.tobytes())
    
    delta_t = t1 - t0 #s
    t_expd = buf_size / 2 / fs #s
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
dt = 1/fs #s
t_arr = np.arange(len(read_data))*dt
plt.plot(t_arr, read_data)

        