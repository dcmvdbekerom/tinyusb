# -*- coding: utf-8 -*-
"""
Created on Sat Sep 26 20:28:21 2026

@author: dcmvd
"""

import struct
import sys
import usb.core
import usb.util

# ==============================================================================
# CONFIGURATION
# Match these exactly to your TinyUSB device descriptors!
# ==============================================================================
VENDOR_ID = 0xCAFE  # Replace with your actual VID (Hex)
PRODUCT_ID = 0x133F  # Replace with your actual PID (Hex)

# Endpoint addresses from your usb_descriptors.c
ENDPOINT_IN = 0x83  # Bulk IN (Device to Host)
ENDPOINT_OUT = 0x03  # Bulk OUT (Host to Device)

TIMEOUT_MS = 5000  # Communication timeout


def main():
    # 1. Find the WCID WinUSB Device
    print(
        f"Searching for USB device (VID: {hex(VENDOR_ID)}, PID: {hex(PRODUCT_ID)})..."
    )
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

    if dev is None:
        print("Device not found! Is it plugged in and bound to WinUSB?")
        sys.exit(1)

    print("Device found successfully.")

    # 2. Handle standard Windows detached driver boilerplate
    # (Usually not required for pure WinUSB, but good practice)
    if dev.is_kernel_driver_active(0):
        try:
            dev.detach_kernel_driver(0)
        except usb.core.USBError as e:
            sys.exit(f"Could not detach kernel driver: {str(e)}")

    # 3. Set the active configuration
    try:
        dev.set_configuration()
    except usb.core.USBError as e:
        sys.exit(f"Failed to set configuration: {str(e)}")

    # ==============================================================================
    # DATA TRANSMISSION (int32_t)
    # ==============================================================================
    # Define an integer array to send to your device
    data_to_send = [42, -100, 2147483647, -2147483648]

    print(f"\n[1] Preparing data to send: {data_to_send}")

    # Pack the list of integers into a binary byte string matching C's int32_t layout.
    # '<i' means: Little-Endian (<), Signed 32-bit Integer (i).
    # Multiplying by len(data_to_send) creates the format specifier string (e.g., '<iiii')
    packed_bytes = struct.pack(f"<{len(data_to_send)}i", *data_to_send)

    try:
        # Write binary data to the Bulk OUT endpoint
        bytes_written = dev.write(ENDPOINT_OUT, packed_bytes, timeout=TIMEOUT_MS)
        print(f"--> Successfully sent {bytes_written} bytes to the device.")

        # ==============================================================================
        # DATA RECEPTION (int32_t)
        # ==============================================================================
        print("\n[2] Reading response from device via Bulk IN...")

        # Read back raw data. We expect the same amount of bytes back.
        # Pass the number of bytes to read as the second argument.
        raw_received = dev.read(ENDPOINT_IN, len(packed_bytes), timeout=TIMEOUT_MS)

        # Convert the raw array/bytes back into readable Python integers
        num_integers_received = len(raw_received) // 4
        unpacked_data = struct.unpack(
            f"<{num_integers_received}i", bytes(raw_received)
        )

        print(f"<-- Successfully received {len(raw_received)} raw bytes.")
        print(f"Decoded values (int32_t list): {list(unpacked_data)}")

    except usb.core.USBError as e:
        print(f"\n[ERROR] USB Communication failed: {e}")

    finally:
        # Clean up and release the interface
        usb.util.dispose_resources(dev)
        print("\nUSB resources released.")


if __name__ == "__main__":
    main()