#!/usr/bin/env python         
# -*- coding: utf-8 -*-

import time
import serial
import rospy
from sensor_msgs.msg import Imu
#from geometry_msgs.msg   import Vector3
from JY901 import *



def main():
    portname = "/dev/wt931"
    baudrate = 921600
    wt931 = CJY901()

    rospy.init_node('wt931_imu', anonymous = False )
    pub = rospy.Publisher("imu", Imu, queue_size=1000)
    #pub_euler = rospy.Publisher("euler", Vector3, queue_size=1000)
    msg = Imu()
    #msg_euler = Vector3()

    #loop_rate = rospy.Rate(100)
    
    try:
        serialPort = serial.Serial(portname, baudrate)
        serialPort.writeTimeout = 2 #写超时
        print ("串口打开成功%s"%portname),
        print (",波特率为%s"%baudrate)
    except Exception as e:
        print ("串口被占用，请重新启动")
    
    t0 = time.time()

    while not rospy.is_shutdown():
        if serialPort.inWaiting() >= 44:
            print("用时%s"%(time.time() - t0))
            t0 = time.time()
            #print("串口缓存数量 %s"%serialPort.inWaiting())
            try:
                rxBuffer = list(serialPort.read(44))
                for i in range(0, 44):
                    wt931.CopeSerialData(rxBuffer[i])
            except Exception as e:
                print("清理串口"),
                serialPort.flush()
                time.sleep(5)
                print("清理串口完毕")
        else:
            pass
        #print("串口缓存数量 %s"%serialPort.inWaiting())

        #时间戳以及坐标系名称
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        #四元素
        msg.orientation.x = wt931.stcQuater.q0
        msg.orientation.y = wt931.stcQuater.q1
        msg.orientation.z = wt931.stcQuater.q2
        msg.orientation.w = wt931.stcQuater.q3
        msg.orientation_covariance = [0,0,0,
                                      0,0,0,
                                      0,0,0]

        #角速度
        msg.angular_velocity.x = wt931.stcGyro.w[0]
        msg.angular_velocity.y = wt931.stcGyro.w[1]
        msg.angular_velocity.z = wt931.stcGyro.w[2]
        msg.angular_velocity_covariance = [0,0,0,
                                           0,0,0,
                                           0,0,0]

        #加速度
        msg.linear_acceleration.x = wt931.stcAcc.a[0]
        msg.linear_acceleration.y = wt931.stcAcc.a[1]
        msg.linear_acceleration.z = wt931.stcAcc.a[2]
        msg.linear_acceleration_covariance = [0,0,0,
                                              0,0,0,
                                              0,0,0]

        #欧拉角
        #msg_euler.x = wt931.stcAngle.Angle[0]
        #msg_euler.y = wt931.stcAngle.Angle[1]
        #msg_euler.z = wt931.stcAngle.Angle[2]

        #发布数据
        pub.publish(msg)
        #pub_euler.publish(msg_euler)
        time.sleep(0.0005)

if __name__ == '__main__':
    main()

