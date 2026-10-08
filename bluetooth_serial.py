"""Communication Bluetooth HC-05"""
import serial,serial.tools.list_ports
class BluetoothSerial:
    def __init__(self,baud=9600):self.baud=baud;self.ser=None
    def connect(self,port=None):
        if not port:
            for p in serial.tools.list_ports.comports():
                if "HC-05" in p.description or "Bluetooth" in p.description:port=p.device;break
        if port:
            try:self.ser=serial.Serial(port,self.baud,timeout=1);return True
            except:return False
        return False
    def is_connected(self):return self.ser and self.ser.is_open
    def send(self,cmd):
        if self.is_connected():self.ser.write((cmd+"\n").encode())
    def receive(self):
        if self.is_connected() and self.ser.in_waiting:return self.ser.readline().decode().strip()
        return None
