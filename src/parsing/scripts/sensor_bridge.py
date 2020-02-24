#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import serial
import sys

# serial port globals
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200

class SerialParser(object):
    def __init__(self):
        # configure and initialize serial port
        self.ser = serial.Serial()
        self.ser.baudrate = SERIAL_BAUD
        self.ser.port = SERIAL_PORT
        self.ser.open()

        self.pub1 = rospy.Publisher('ir1', String, queue_size=10)
        self.pub2 = rospy.Publisher('ir2', String, queue_size=10)
        self.pub3 = rospy.Publisher('ir3', String, queue_size=10)
        self.pub4 = rospy.Publisher('ir4', String, queue_size=10)
        self.pub5 = rospy.Publisher('ir5', String, queue_size=10)
        self.pub6 = rospy.Publisher('ir_all', String, queue_size=10)

        rospy.init_node('talker', anonymous=True)
        # rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            self.read_vals()
            # rate.sleep()            

    def parse_message(self, msg):
        msg = msg.strip()
        vals = msg.split(' ')
        return vals
    
    def read_vals(self):
        msg = self.ser.readline()
        parsed = msg.decode("utf-8").replace("\r\n", "\n")
        vals = self.parse_message(parsed)

        if len(vals) < 5: return
        
        self.pub1.publish(vals[0])
        self.pub2.publish(vals[1])
        self.pub3.publish(vals[2])
        self.pub4.publish(vals[3])
        self.pub5.publish(vals[4])
        self.pub6.publish(parsed)

if __name__ == '__main__':
    parser = SerialParser()