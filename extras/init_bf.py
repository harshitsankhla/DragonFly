import serial
from serial.tools import list_ports
import re
import subprocess
import time


class TimeLimitExpired(object):
    pass


class Arduino:

    def __init__(self):
        self.keyword = ['Arduino', '340']
        self.device_re = re.compile(
            "Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
        self.df = subprocess.check_output("lsusb").decode().split('\n')
        self.dinfo = []
        self.arduino = self.find_arduino()

    def find_arduino(self):
        serial_device = None
        for i in self.df:
            if self.keyword[1] in i:
                self.info = self.device_re.match(i)
                self.dinfo = self.info.groupdict()
                print('Detected =', self.dinfo['id'])
                break

        if self.dinfo:
            self.ms = list_ports.comports()
            for m in self.ms:
                print(m)
                if self.dinfo['id'].upper() in m.hwid:
                    print('found')
                    while 1:
                        try:
                            serial_device = serial.Serial(m.device, 9600)
                            time.sleep(2)
                            serial_device.write(b'0')
                            print(m.device)
                            break
                        except Exception as e:
                            print e
                            break
        else:
            print('No Arduino Detected')
        return serial_device

    def get_data_from_arduino_helper(self, arduino_instance):

        import threading

        class FuncThread(threading.Thread):

            def __init__(self):
                threading.Thread.__init__(self)
                self.result = None

            def run(self):
                while True:
                    val = str(arduino_instance.readline())
                    weight = val.split("\\")[0]
                    break
                return weight

        it = FuncThread()
        it.start()
        it.join(2)
        if it.isAlive():
            raise TimeLimitExpired
        else:
            return it.result