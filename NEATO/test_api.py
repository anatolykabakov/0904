
import serial
import time
def read_all(port, chunk_size=200):
       """Read all characters on the serial port and return them."""
       if not port.timeout:
           raise TypeError('Port needs to have a timeout set!')
       read_buffer = b''
       while True:
           # Read in chunks. Each chunk will wait as long as specified by
           # timeout. Increase chunk_size to fail quicker
           byte_chunk = port.read(size=chunk_size)
           read_buffer += byte_chunk
           if not len(byte_chunk) == chunk_size:
               break
       return read_buffer

def get_encoders(neato):
    #neato.flush()
    neato.write('GetMotors\r\n'.encode())
    #time.sleep(0.1)
    #neato.read(val)
    line = b''
    data = neato.read(1)
    while True:
        data = neato.read()
        #if data == b'':
        #    time.sleep('0.01')
        line+=data
        print(line)
    return line

def f1(neato):
    enc = []
    neato.write('getmotors\r\n'.encode())
    time.sleep(1)
    line = neato.readline().decode()
    print(line)
    while line.split(',')[0]!='Parameter':
        time.sleep(0.1)
        line = neato.readline().decode()
        print(line)
        
    for i in range(16):
        values = neato.readline().decode().split(',')
        enc.append(value)
        print(value)
    return enc


if __name__ == '__main__':
    neato = serial.Serial('/dev/ttyACM1',115200)
    neato.write('testmode on\r\n'.encode())
    #neato.write('setldsrotation on\r\n'.encode())
    #neato.write('getmotors\m'.encode())
    #data = read_all(neato)
    data = get_encoders(neato)
    #data = f1(neato)
    print(data)
 
    
    
