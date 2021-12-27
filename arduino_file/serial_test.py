import serial
import time

def serial_test():
    ser = serial.Serial(
        port='/dev/ttyACM0',\
        baudrate=9600,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=0)

    print("connected to: " + ser.portstr)

    #this will store the line
    line = []

    while True:
        for c in ser.read():
            line.append(c)
            byte_bit = bin(c)
            print(c)
            time.sleep(1)
            
            
            

    ser.close()
if __name__ == "__main__":
    try:
        serial_test()
    except:
        print("Check Port")
