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
    send_arr = np.arange(1, 11, dtype="<i4")  # 10 little-endian int32s
    print(f"Writing: {send_arr}")
    ep_out.write(send_arr.tobytes())

    print("Reading response...")
    raw = ep_in.read(send_arr.nbytes, timeout=1000)
    recv_arr = np.frombuffer(bytes(raw), dtype="<i4")
    print(f"Received: {recv_arr}")

    expected = send_arr + 1
    if np.array_equal(recv_arr, expected):
        print("Loopback OK: all values incremented correctly.")
    else:
        print(f"Mismatch! Expected {expected}, got {recv_arr}")

finally:
    usb.util.dispose_resources(dev)
    print("Connection closed.")