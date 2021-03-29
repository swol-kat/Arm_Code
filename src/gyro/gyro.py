import serial
import adafruit_bno055
from multiprocessing import Process, Pipe
import time

def gyro_worker(conn, port):
    uart = serial.Serial(port)
    sensor = adafruit_bno055.BNO055_UART(uart)
    conn.send(0)
    while True:
        out_dict = {} #do we need any other gyro data?
        out_dict['euler'] = sensor.euler
        out_dict['gravity'] =  sensor.gravity
        conn.send(0)
        if conn.poll(0.05): #if something is sent then the thread closes
            return


class gyro:
    def __init__(self, port = "/dev/ttyUSB1"):
        self.port = port
        self.thread_running = False
        self.last_values = {}
        self.thread = None
        self.conn = None

    def start_thread(self):
        to_worker, to_object = Pipe()
        self.thread = Process(target=gyro_worker, args=(to_worker, self.port, ))
        self.thread_running = True
        self.conn = to_object
        started = self.conn.recv()

    def get_values(self): #this is theoretically called ~60hz for newest values
        if self.conn != None:
            while self.conn.poll():
                self.last_values = self.conn.recv()
        if self.thread_running:
            return self.last_values
        return None
    
    def stop_thread(self):
        self.thread_running = False
        self.conn.send(0) #any data sent closes connection
        self.thread.join()
        self.conn.close()
        self.conn = None
        self.thread = None