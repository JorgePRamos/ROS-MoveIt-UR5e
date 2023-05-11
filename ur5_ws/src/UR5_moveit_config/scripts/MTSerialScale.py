#!/usr/bin/env python

import serial
import re
import time
from datetime import date

import sys
import rospy
from std_msgs.msg import String
import std_msgs.msg
from sensor_msgs.msg import Illuminance

class SerialPacket:
    def __init__(self):
        self.Sent = ""
        self.Received = ""
        self.Log = ""
        self.IsError = False
        self.DataSent = date.today()

class MTSerialScale:
    def __init__(self):
        self.COMPort = "/dev/ttyUSB0"
        self.BaudRate = 9600
        self.ByteSize = 8
        self.Parity = 'N'
        self.StopBits = 1
        self.Packets = []
        self.ResponseTermination = '\r\n'
        self.Weight = 0
        self.LastWeightTimestamp = None
        self.ZeroScaleFlag = False

        # NODES
        rospy.loginfo('Starting MT Scale node')
        rospy.init_node('MTScale', anonymous=True)
        rospy.Subscriber('MTScaleCommands',String,self.command_received_handler)
        self.WeightPublisher = rospy.Publisher('MTScaleOutput', Illuminance, queue_size=10)
        rospy.loginfo('MT Init Completed')

    def command_received_handler(self, data):
        #rospy.loginfo('Scale commands received: {}'.format(data))
        if(data.data=="zero_scale"):
            self.ZeroScaleFlag = True
            rospy.loginfo('MTScale ZeroScale command received')
        else:
            rospy.logerr('Unknown command received: [{}]'.format(data.data))

    def GetWeight(self):
        msg = 'SI\r\n'
        status, response = self.SendAndReceive(msg)
        if(status):
            if(self.DecodeResponse(response)):
                return True
            else:
                print('SUCCESS BUT response was not decoded: {}'.format(response))
        else:
            print('FAILED response: {}'.format(response))

    def ZeroScale(self):
        msg = 'ZI\r\n'
        status, response = self.SendAndReceive(msg)
        if(status):
            if(self.DecodeResponse(response)):
                return True
            else:
                print('SUCCESS BUT response was not decoded: {}'.format(response))
        else:
            print('FAILED response: {}'.format(response))

    def DecodeResponse(self, response):
        if (response is not None) & (len(response)>0):
            
            # chec if it contains a weight
            mW = re.compile('[-]?[0-9]+\.[0-9]+ g')
            values = mW.findall(response)
            if(values is not None) & (len(values)>0):
                w = values[0].replace(' g','')
                self.Weight = float(w)
                self.LastWeightTimestamp = time.time()
                return True
            elif(response=='ZI D'):
                return True
            elif(response=='ZI S'):
                return True
            else:
                print('Response [{}] cannot be decocded'.format(response))
                return False
        else:
            return False

    def SendAndReceive(self, msg, maxWaitTimeMsec=500):
        ser = serial.Serial(self.COMPort)
        ser.BaudRate = self.BaudRate
        ser.ByteSize = self.ByteSize
        ser.Parity = self.Parity
        ser.StopBits = self.StopBits

        pck = SerialPacket()
        self.Packets.append(pck)

        pck.Sent = msg

        try:
            if(ser.closed):
                ser.open()

            ser.write(msg)
            start_time = time.time()
            buffer = ""
            response = ""
            time_exceeded = False
            response_received = False

            while (not time_exceeded) & (not response_received):
                # read buffer
                sByte =  ser.read()
                if(sByte is not None):
                    # convert to utf-8
                    sUtf  = sByte.decode('utf-8')
                    buffer = buffer + sUtf
                # Check if response has been received
                if(len(buffer)>2):
                    # get last two characters
                    termination = buffer[len(buffer)-2:len(buffer)]
                    if(termination==self.ResponseTermination):
                        response = buffer[0:len(buffer)-2]
                        response_received = True
                        pck.Received = response

                elapsed_time = time.time() - start_time
                if((elapsed_time*1000) > maxWaitTimeMsec):
                    time_exceeded = True

            if len(response)>0:
                return True,response
            else:
                return False,buffer

        except Exception as e:
            pck.Log = 'Exception: {}'.format(e)
            pck.IsError = True
            return False,pck.Log

        finally:
            if not ser.closed:
                ser.close()

    def Start(self):
        rospy.loginfo('Starting MT reading process')
        while not rospy.is_shutdown():
            
            if(self.ZeroScaleFlag):
                rospy.loginfo('Zeroeing scale...')
                if(self.ZeroScale()):
                    rospy.loginfo('  --> Zero scale command sent')
                    self.ZeroScaleFlag = False
                else:
                    rospy.logerr('  --> Zero scale command FAILED')

            # read the weight
            if(self.GetWeight()):
                # publish result
                ill = Illuminance()
                ill.header = std_msgs.msg.Header()
                ill.header.stamp = rospy.Time.now()
                ill.illuminance = self.Weight
                self.WeightPublisher.publish(ill)
            rospy.sleep(0.25)

if __name__ == '__main__':
    s = MTSerialScale()
    
    #if(s.GetWeight()):
    #    print('Latest weight: {}'.format(s.Weight))

    s.Start()

