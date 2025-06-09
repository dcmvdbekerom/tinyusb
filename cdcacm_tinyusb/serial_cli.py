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

def read_response(ser):
    time.sleep(0.01)
    response = b''
    while ser.in_waiting:
        response += ser.read_all()
    return response.decode('utf-8', errors='ignore')


with serial.Serial(com_port, 115200, timeout=1) as ser:

    # Send handshake
    print('MCU repsonse:',end=' ')
    ser.write(b'BTLDCMD0\n')
    response = read_response(ser)
    print(response)
    # ser.write(b'BTLDCMD1\n')
    
    # Reset pages
    print('Resetting pages...', end=' ')
    ser.write(b'BTLDCMD1\n')
    print('Done!')    
    
    # print('\nWrite next page with 0xEE...')
    # ser.write(b'BTLDCMD3\n')
    # print('Done!')
    # response = ''
    # while response == '':
    #     response = read_response(ser)
    # print('MCU repsonse:', response)
    
    # Write data
    dword = 0xDEADBEEF01234567
    rowData = dword.to_bytes(8, byteorder='big') * 32 #32 dwords per row
    
    for row in range(8):
        print('Writing data row={:d}...'.format(row), end = ' ')
        ser.write(rowData)
        print('Done!')
        
    # Reset MCU
    print('Resetting MCU...', end=' ')
    ser.write(b'BTLDCMD2\n')
    print('Done!')
    
    
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

