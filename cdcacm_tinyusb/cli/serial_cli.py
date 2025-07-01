# -*- coding: utf-8 -*-
"""
Created on Sat May 31 10:06:48 2025

@author: dcmvd
"""

import serial
import serial.tools.list_ports
import time 

PAGE_SIZE = 2048
BUF_SIZE = 64

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


with serial.Serial(com_port, 115200, timeout=1) as ser, open('miniblink.bin', 'rb') as f:

    # Send id request
    print('MCU repsonse:',end=' ')
    ser.write(b'BTLDCMD0\n')
    response = read_response(ser)
    print(response)
    
    # Reset pages
    print('Resetting pages...', end=' ')
    ser.write(b'BTLDCMD1\n')
    print('Done!')    
    
    
    while True:
        
        data = f.read(PAGE_SIZE)          
        
        # dword = 0x01234567_89ABCDEF_00112233
        # data = dword.to_bytes(12, byteorder='little')*3
        
        print(f'Writing {len(data):d} bytes...')
        ser.write(data)
        
        if len(data) != PAGE_SIZE:

            print('Resetting MCU...', end=' ')
            
            res = len(data) % BUF_SIZE
            cmd2 = b'BTLDCMD2' + res.to_bytes(1)
            ser.write(cmd2)
            print('Done!')   
            
            break
