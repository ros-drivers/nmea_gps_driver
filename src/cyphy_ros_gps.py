#!/usr/bin/env python

import roslib
roslib.load_manifest('cyphy_ros_gps')
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from cyphy_ros_gps.msg import GPStime

import serial, string, math
import decimal as dec

if __name__ == "__main__":
    #init publisher
    gpspub = rospy.Publisher('GPS', NavSatFix)
    gpsVelPub = rospy.Publisher('GPSVel',TwistStamped)
    gpstimePub = rospy.Publisher('GPStime', GPStime)
    rospy.init_node('GPSNode')
    #Init GPS port
    GPSport = rospy.get_param('~port','/dev/ttyUSB0')
    GPSrate = rospy.get_param('~rate',4800)
    frame_id = rospy.get_param('~frame_id','GPS')
    navData = NavSatFix()
    gpsVel = TwistStamped()
    gpstime = GPStime()
    navData.header.frame_id = frame_id
    gpsVel.header.frame_id = frame_id
    gpstime.header.frame_id = frame_id
    GPSLock = False
    try:
        GPS = serial.Serial(port=GPSport, baudrate=GPSrate, timeout=2)
        #Read in GPS
        while not rospy.is_shutdown():
            #read GPS line
            data = GPS.readline()
            timeNow = rospy.get_rostime()
            fields = data.split(',')
            for i in fields:
                i = i.strip(',')
            #Check for satellite lock
            if fields[0].find('GSA') > 0:
                lockState = int(fields[2])
                #print 'lockState=',lockState
                if lockState == 3:
                    GPSLock = True
                else:
                    GPSLock = False
            #if not satellite lock message parse it separately
            else:
                if GPSLock == True:
                    if fields[0].find('RMC') > 0:
                        #print fields
                        gpsVel.header.stamp = timeNow
                        gpsVel.twist.linear.x = float(fields[7])*0.514444444444*math.sin(float(fields[8]))
                        gpsVel.twist.linear.y = float(fields[7])*0.514444444444*math.cos(float(fields[8]))
                        gpsVelPub.publish(gpsVel)
                    
                        navData.status.status = GPSLock-1
                        navData.header.stamp = gpsVel.header.stamp
                        navData.status.service = 1
                        
                        gpstime.header.stamp = gpsVel.header.stamp
                        gpstime.gpstime = rospy.Time(float(fields[1]))

                        longitude = dec.Decimal('%.10f' % (dec.Decimal(fields[5][0:3]) + dec.Decimal(fields[5][3:fields[5].__len__()]) / 60))
                        if fields[6] == 'W':
                            longitude = -longitude

                        latitude = dec.Decimal('%.10f' % (dec.Decimal(fields[3][0:2]) + dec.Decimal(fields[3][2:fields[3].__len__()]) / 60))
                        if fields[4] == 'S':
                            latitude = -latitude

                        #publish data
                        navData.latitude = latitude
                        navData.longitude = longitude
                        navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                        gpspub.publish(navData)
                        gpstimePub.publish(gpstime)
          
                else:
                    pass
                    #print data

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial port
