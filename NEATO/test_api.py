
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
    #neato.write('GetMotors\n'.encode())
    line = b''
    data = neato.read(2)
    while data != b'\x1a\x1a':
        data = neato.read(2)
        line+=data
        
    return line

if __name__ == '__main__':
    neato = serial.Serial('/dev/ttyACM3',115200,serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE,0.1)
    neato.write('testmode on\n'.encode())
    neato.write('setldsrotation on\n'.encode())
    neato.write('getmotors\m'.encode())
    data = read_all(neato)
    print(data)
 
    
    
