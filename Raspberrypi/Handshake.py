import serial

def Handshake():
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    #ser.write("testing serial connection\n".encode('utf-8'))
    #ser.write("sending via RPi\n".encode('utf-8'))

    hello = 18  #hello is 0x12
    extend = 0
    hello_b = hello.to_bytes(1, byteorder='big')
    extend_b = extend.to_bytes(4, byteorder='big')
    send_str = str(hello_b) + str(extend_b)
    ser.write("1230FE3030".encode('utf-8'))#send hello
    try:
        while 1:
            ack = 17  #ack is 0x11
            ack_b = int(ack).to_bytes(1, byteorder = 'big')
            response = ser.readline()
            print(str(response)+ " " + str(type(response)))
            print(str(response,'utf-8') + str(type(str(response,'utf-8'))))
            if (str(response,'utf-8') == str(ack)):  #received the 1st ack
                print("Arduino online")
                send_str = str(ack_b) + str(extend_b)
                ser.write("1100000000".encode('utf-8')) #send the 2nd ack
                print('write back ack!')
                return 1
                break        
    except KeyboardInterrupt:
        ser.close()
        return 0

if __name__ == '__main__':
    Handshake()