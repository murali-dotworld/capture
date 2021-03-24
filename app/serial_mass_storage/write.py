import serial
ser =  serial.Serial('COM5', 115200)
ser.write('capture'.encode())