import serial 

messageStarted = 0
dataBuffer= [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
dataBufferIndex = 0


ser = serial.Serial(
    port='/dev/ttyACM0',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
            timeout=0)    #init setup for --> 1.Serial port 2.baudrate etc.
print("connected to: " + ser.portstr) #to check port connected
    
while True:
    if ser.in_waiting:  # Or: while ser.inWaiting(): --> wait for serial communication
        for c in ser.read():
            
            byte_bit = bin(c)
            #print(c)
            read = c
           # print ser.readline()
        dataBuffer[dataBufferIndex] = read
        if not messageStarted:
            
            dataBufferIndexIncrease = 1
            if dataBufferIndex > 1:
                if dataBuffer[dataBufferIndex - 1] == 253 and dataBuffer[dataBufferIndex] == 255:
                    messageStarted = 1
                    dataBufferIndex = 0
                    dataBufferIndexIncrease = 0
                        
            if(dataBufferIndexIncrease):
                dataBufferIndex = dataBufferIndex + 1
        else:
            
            if dataBufferIndex == 4:
                sum = dataBuffer[4] | dataBuffer[5]<<8
                data1 = dataBuffer[0] | dataBuffer[1]<<8
                if (dataBuffer[0] + dataBuffer[1] + dataBuffer[2] + dataBuffer[3] == sum ):
                    data0 = dataBuffer[0]
                    #data1 = dataBuffer[1]
                    print(str(bin(dataBuffer[0])) + ' and ' + str(bin(dataBuffer[1]))+ ' and ' + str(bin(data1)))
                    print(str(dataBuffer[0]) + ' and ' + str(dataBuffer[1])+ ' and ' + str(data1))
                    #print('true '+str(dataBuffer[0] | dataBuffer[1]<<8) + ' and ' + str(dataBuffer[2]) + ' and ' + str(dataBuffer[3]) + ' and ' + str(dataBuffer[4])+ ' and ' + str(dataBuffer[5]))
                    #print('true ' + str(bin(dataBuffer[0])) + ' and ' + str(bin(dataBuffer[1])) + ' and ' +  str(bin(data1)))
                    #print(str(bin(dataBuffer[3]))+ ' and ' + str(bin(dataBuffer[4])) + ' and ' +  str(dataBuffer[1]+dataBuffer[2]+dataBuffer[3])+ ' and ' +  str(bin(sum)))
                else:
                    print('Error reading data fail on check sum')
                    #print(str(dataBuffer[0]) + ' and ' + str(dataBuffer[1]) + ' and ' + str(dataBuffer[2]) + ' and ' + str(bin(dataBuffer[3]))+ ' and ' + str(bin(dataBuffer[4])) + ' and ' +  str(dataBuffer[1]+dataBuffer[2]+dataBuffer[3])+ ' and ' +  str(bin(sum)))
                    #print('false '+str(dataBuffer[0] | dataBuffer[1]<<8) + ' and ' + str(dataBuffer[2]) + ' and ' + str(dataBuffer[3]) + ' and ' + str(dataBuffer[4])+ ' and ' + str(dataBuffer[5]))
                    #print('false ' + str(dataBuffer[4]) + ' and ' + str(dataBuffer[5]) + ' and ' +  str(bin(data1)))
                    #print(str(dataBuffer[3]) + ' and ' + str(dataBuffer[4]) + ' and '+ str(dataBuffer[3]+dataBuffer[4]))
                messageStarted = 0
            dataBufferIndex = dataBufferIndex + 1
                

            