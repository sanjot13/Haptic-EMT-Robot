"""Prints the readings of a Bota Systems Serial sensor.

Usage: python bota_serial_example.py <port>

This example expects a device layout according to
_expected_device_layout, see below.
"""

import sys
import struct
import time
import threading

from collections import namedtuple

import serial
from crc import CrcCalculator, Configuration


class SerialExample:

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 512
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01;

    def __init__(self, port):
        self._port = port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def bota_sensor_setup(self):
        print("bota_sensor_setup")
        # Wait for streaming of data
        self._ser.read_until(bytes('App Init', 'ascii'))
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,C,0', 'ascii'))

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        time_step = 0.00001953125*self.SINC_LENGTH
        print("Timestep: {}".format(time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,f,0', 'ascii'))

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,R,0', 'ascii'))

    def _processdata_thread(self):
        while not self._pd_thread_stop_event.is_set():
            frame_synced = False
            crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
            crc_calculator = CrcCalculator(crc16X25Configuration)

            while not frame_synced and not self._pd_thread_stop_event.is_set():
                possible_header = self._ser.read(1)
                if self.FRAME_HEADER == possible_header:
                    #print(possible_header)
                    data_frame = self._ser.read(34)
                    crc16_ccitt_frame = self._ser.read(2)

                    #status = struct.unpack_from('H', data_frame, 0)[0]
                    #print("Status {}".format(status))

                    #Fx = struct.unpack_from('f', data_frame, 2)[0]
                    #print("Fx {}".format(Fx))
                    #Fy = struct.unpack_from('f', data_frame, 6)[0]
                    #print("Fy {}".format(Fy))
                    #Fz = struct.unpack_from('f', data_frame, 10)[0]
                    #print("Fz {}".format(Fz))
                    #Mx = struct.unpack_from('f', data_frame, 14)[0]
                    #print("Mx {}".format(Mx))
                    #My = struct.unpack_from('f', data_frame, 18)[0]
                    #print("My {}".format(My))
                    #Mz = struct.unpack_from('f', data_frame, 22)[0]
                    #print("Mz {}".format(Mz))

                    #timestamp = struct.unpack_from('I', data_frame, 26)[0]
                    #print("Timestamp {}".format(timestamp))

                    #temperature = struct.unpack_from('f', data_frame, 30)[0]
                    #print("Temperature {}\n".format(temperature))

                    crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                    # print("crc16_ccitt {}".format(crc16_ccitt)) 
                    checksum = crc_calculator.calculate_checksum(data_frame)
                    # print("checksum {}".format(checksum))
                    if checksum == crc16_ccitt:
                        print("Frame synced")
                        frame_synced = True;
                    else:
                        self._ser.read(1)

            while frame_synced and not self._pd_thread_stop_event.is_set():            
                start_time = time.perf_counter()
                frame_header = self._ser.read(1)

                if frame_header != self.FRAME_HEADER:
                    print("Lost sync")
                    frame_synced = False
                    break

                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.calculate_checksum(data_frame)
                if checksum != crc16_ccitt:
                    print("CRC mismatch received")
                    break

                status = struct.unpack_from('H', data_frame, 0)[0]
                print("Status {}".format(status))

                Fx = struct.unpack_from('f', data_frame, 2)[0]
                print("Fx {}".format(Fx))
                Fy = struct.unpack_from('f', data_frame, 6)[0]
                print("Fy {}".format(Fy))
                Fz = struct.unpack_from('f', data_frame, 10)[0]
                print("Fz {}".format(Fz))
                Mx = struct.unpack_from('f', data_frame, 14)[0]
                print("Mx {}".format(Mx))
                My = struct.unpack_from('f', data_frame, 18)[0]
                print("My {}".format(My))
                Mz = struct.unpack_from('f', data_frame, 22)[0]
                print("Mz {}".format(Mz))

                timestamp = struct.unpack_from('I', data_frame, 26)[0]
                print("Timestamp {}".format(timestamp))

                temperature = struct.unpack_from('f', data_frame, 30)[0]
                print("Temperature {}\n".format(temperature))
                
                time_diff = time.perf_counter() - start_time
                if time_diff < self.time_step:
                    SerialExample._sleep((self.time_step - time_diff)/2)
 

    def _my_loop(self):

        try:
            while 1:
                print('Run my loop')

                time.sleep(1.0)

        except KeyboardInterrupt:
            # ctrl-C abort handling
            print('stopped')


    def run(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
        except:
            raise SerialExampleError('Could not open port')

        if not self._ser.is_open:
            raise SerialExampleError('Could not open port')

        self.bota_sensor_setup()

        #check_thread = threading.Thread(target=self._check_thread)
        #check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()
        

        device_running = True

        if device_running:
            self._my_loop()

        self._pd_thread_stop_event.set()
        proc_thread.join()
        #check_thread.join()

        self._ser.close()

        if not device_running:
            raise SerialExampleError('Device is not running')

    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

class SerialExampleError(Exception):
    def __init__(self, message):
        super(SerialExampleError, self).__init__(message)
        self.message = message


if __name__ == '__main__':

    print('bota_serial_example started')

    if len(sys.argv) > 1:
        try:
            SerialExample(sys.argv[1]).run()
        except SerialExampleError as expt:
            print('bota_serial_example failed: ' + expt.message)
            sys.exit(1)
    else:
        print('usage: bota_serial_example portname')
        sys.exit(1)
