# -*- coding: utf-8 -*-
"""
Created on Sat May 31 10:06:48 2025

@author: dcmvd
"""

import serial
import serial.tools.list_ports
import time 

def find_first_active_com_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        try:
            with serial.Serial(port.device, 115200, timeout=1) as ser:
                print(port)
                return port.device
        except (OSError, serial.SerialException):
            continue
    return None

global response
com_port = find_first_active_com_port()
if not com_port:
    print("No active COM port found.")

with serial.Serial(com_port, 115200, timeout=1) as ser:
    # ser.write(b'BTLDCMD1\n')
    
    ser.write(b'BTLDCMD1\n')
    # ser.write(b'BTLDCMD1\n')
    
    
    time.sleep(0.01)
    response = b''
    while ser.in_waiting:
        response += ser.read_all()
    print(response.decode('utf-8', errors='ignore'))
    
    
    
    
    # pattern = 0xDEADBEEF
    # pageData = pattern.to_bytes(4, byteorder='big') * 512
    
    # ser.write(pageData)
    # time.sleep(0.01)
    # response = b''
    # while ser.in_waiting:
    #     response += ser.read_all()
    # print(response.decode('utf-8', errors='ignore'))
            
    # ser.write(b'BTLDCMD2\n')
    





# try:
#     with serial.Serial(com_port, 115200, timeout=1) as ser:
#         ser.write(b'BTLDCMD1\n')  # Sending command
#         time.sleep(0.01)
#         response = b''
#         while ser.in_waiting:
#             response += ser.read_all()
        
#         print(response.decode('utf-8', errors='ignore'))
        
#         print('Done!')
        

# except Exception as e:
#     print(f"Error communicating with COM port: {e}")

