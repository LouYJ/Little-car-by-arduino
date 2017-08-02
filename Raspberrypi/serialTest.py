import serial

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

ser.write("testing serial connection\n".encode('utf-8'))
ser.write("sending via RPi\n".encode('utf-8'))
input_value = input('please input some message')
ser.write(input_value.encode('utf-8'))
try:
    while 1:
        response = ser.readline()
        print (response)

        
except KeyboardInterrupt:
    ser.close()
