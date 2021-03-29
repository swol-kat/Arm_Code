import controller_parent
import serial
import time

def rc_worker(conn, port):
    ser = serial.Serial(
    port = port , baudrate = 125000,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE)

class rc_controller(controller_parent):
    def __init__(self, port):
        Parent.__init__()
        self.port = port
        self.thread = None
        self.conn = None
    
    def start_worker(self):
        #implement the rest of this
        pass
    
    def get_variables(self):
        out_dict = Parent.get_variables()