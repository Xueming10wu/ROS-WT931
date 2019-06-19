#!/usr/bin/env python         
# -*- coding: utf-8 -*-
import os

class STime(object):
    #时间输出
    ucYear = ""
    strucMonth = ""
    strucDay = ""
    strucHour = ""
    strucMinute = ""
    strucSecond = ""
    usMiliSecond = 0.0

class SAcc(object):
    #加速度输出
    a = [0.0 for i in range(3)]
    T = 0.0

class SGyro(object):
    #角速度输出
    w = [0.0 for i in range(3)]
    T = 0.0

class SAngle(object):
    #角度输出
    Angle = [0.0 for i in range(3)]
    T = 0.0

class SMag(object):
    #磁场输出
    h = [0.0 for i in range(3)]
    T = 0.0

class SDStatus(object):
    #端口状态数据输出
    sDStatus = [0.0 for i in range(4)]

class SPress(object):
    #气压、高度输出
    lPressure = 0.0
    lAltitude = 0.0

class SLonLat(object):
    #经纬度输出
    lLon = 0.0
    lLat = 0.0

class SGPSV(object):
    #地速输出
    sGPSHeight = 0.0
    sGPSYaw = 0.0
    lGPSVelocity = 0.0

class SQuater(object):
    #四元素输出
	q0 = 0.0
	q1 = 0.0
	q2 = 0.0
	q3 = 0.0

class SSN(object):
    #卫星定位精度输出
	sSVNum = 0.0
	sPDOP = 0.0
	sHDOP = 0.0
	sVDOP = 0.0

class CJY901(object):
    def __init__(self):
        self.__key = 0
        self.__rxBuffer = [  0 for i in range(11) ]
        self.stcTime = STime()
        self.stcAcc = SAcc()
        self.stcGyro = SGyro()
        self.stcAngle = SAngle()
        self.stcMag = SMag()
        self.stcDStatus = SDStatus()
        self.stcPress = SPress()
        self.stcLonLat = SLonLat()
        self.stcGPSV = SGPSV()
        self.stcQuater = SQuater()
        self.stcSsn = SSN()
        print("WT931传感器准备就绪")
        
    def CopeSerialData(self, ucData):
        #把数据放到各个结构体中
        self.__rxBuffer[self.__key] = ord(ucData)
        #记录数组位置
        self.__key += 1
        if self.__rxBuffer[0]!=0x55:
            #print("not 0x55")
            #如果发现第一个数据不是界符的话，就将索引置0
            self.__key = 0
            return
        if self.__key < 11:
            #如果数据索引没有到11的话，退出并再次执行本函数
            return
        else:
            #print (hex(self.__rxBuffer[1]))
            ##如果数据索引到11的话，给结构体进行赋值
            if self.__rxBuffer[1] == 0x50:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")   
                    self.stcTime.ucYear = self.__rxBuffer[2]
                    self.stcTime.strucMonth = self.__rxBuffer[3]
                    self.stcTime.strucDay = self.__rxBuffer[4]
                    self.stcTime.strucHour = self.__rxBuffer[5]
                    self.stcTime.strucMinute = self.__rxBuffer[6]
                    self.stcTime.strucSecond = self.__rxBuffer[7]
                    self.stcTime.usMiliSecond = (ord(self.__rxBuffer[9]) << 8 ) | ord(self.__rxBuffer[8])
                else:
                    pass
                    #print("wrong data")    
                    
            
            elif self.__rxBuffer[1] == 0x51:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    g = 9.8
                    self.stcAcc.a[0] = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) ) / 32768*16*g
                    self.stcAcc.a[1] = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 32768*16*g
                    self.stcAcc.a[2] = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) ) / 32768*16*g
                    self.stcAcc.T = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) /100
                    '''
                    print("stcAcc"),
                    print("%.2f" %self.stcAcc.a[0]),
                    print("%.2f" %self.stcAcc.a[1]),
                    print("%.2f" %self.stcAcc.a[2]),
                    print("          %.2f" %self.stcGyro.T)
                    '''
                else:
                    pass
                    #print("wrong data")                   

            elif self.__rxBuffer[1] == 0x52:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")
                    self.stcGyro.w[0] = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) ) / 32768*2000/180
                    self.stcGyro.w[1] = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 32768*2000/180
                    self.stcGyro.w[2] = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) ) / 32768*2000/180
                    self.stcGyro.T = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) /100
                    '''
                    print("stcGyro"),
                    print("%.2f" %self.stcGyro.w[0]),
                    print("%.2f" %self.stcGyro.w[1]),
                    print("%.2f" %self.stcGyro.w[2]),
                    print("          %.2f" %self.stcGyro.T)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x53:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")
                    PI = 3.1415926
                    degree = PI/180
                    self.stcAngle.Angle[0] = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) ) / 32768*180 * degree 
                    self.stcAngle.Angle[1] = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 32768*180 * degree 
                    self.stcAngle.Angle[2] = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) ) / 32768*180 * degree 
                    self.stcAngle.T = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) /100
                    '''
                    print("stcAngle"),
                    print("%.2f" %self.stcAngle.Angle[0]),
                    print("%.2f" %self.stcAngle.Angle[1]),
                    print("%.2f" %self.stcAngle.Angle[2]),
                    print("          %.2f" %self.stcAngle.T)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x54:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")                
                    self.stcMag.h[0] = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) )
                    self.stcMag.h[1] = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) )
                    self.stcMag.h[2] = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) )
                    self.stcMag.T = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) /100
                    '''
                    print("stcMag"),
                    print(self.stcMag.h),
                    print(self.stcMag.T)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x55:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")                   
                    self.stcDStatus.sDStatus[0] = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) )
                    self.stcDStatus.sDStatus[1] = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) )
                    self.stcDStatus.sDStatus[2] = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) )
                    self.stcDStatus.sDStatus[3] = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) )
                    '''
                    print("stcDStatus"),
                    print(self.stcDStatus.sDStatus)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x56:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")   
                    self.stcPress.lPressure = self.ComplementToOriginal(((self.__rxBuffer[5] << 24)|(self.__rxBuffer[4] << 16)|(self.__rxBuffer[3] << 8)|self.__rxBuffer[2]), 32)
                    self.stcPress.lAltitude = self.ComplementToOriginal(((self.__rxBuffer[9] << 24)|(self.__rxBuffer[8] << 16)|(self.__rxBuffer[7] << 8)|self.__rxBuffer[6]), 32)
                    '''
                    print("stcPress"),
                    print(self.stcPress.lPressure),
                    print(self.stcPress.lAltitude)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x57:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")                   
                    self.stcLonLat.lLon = float(self.ComplementToOriginal(((self.__rxBuffer[5] << 24)|(self.__rxBuffer[4] << 16)|(self.__rxBuffer[3] << 8)|self.__rxBuffer[2]), 32)/10000000)
                    self.stcLonLat.lLat = float(self.ComplementToOriginal(((self.__rxBuffer[9] << 24)|(self.__rxBuffer[8] << 16)|(self.__rxBuffer[7] << 8)|self.__rxBuffer[6]), 32)/10000000)
                    '''
                    print("stcLonLat"),
                    print(self.stcLonLat.lLon),
                    print(self.stcLonLat.lLat)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x58:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")   
                    self.stcGPSV.sGPSHeight = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) ) / 10
                    self.stcGPSV.sGPSYaw = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 10
                    self.stcGPSV.lGPSVelocity = float(self.ComplementToOriginal (((self.__rxBuffer[9] << 24)| (self.__rxBuffer[8] << 16)| (self.__rxBuffer[7] << 8)| self.__rxBuffer[6] ), 32))/1000
                    '''
                    print("stcGPSV"),
                    print(self.stcGPSV.sGPSHeight),
                    print(self.stcGPSV.sGPSYaw)
                    print(self.stcGPSV.lGPSVelocity)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x59:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data") 
                    self.stcQuater.q0 = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) ) / 32768
                    self.stcQuater.q1 = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 32768
                    self.stcQuater.q2 = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) ) / 32768
                    self.stcQuater.q3 = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) / 32768
                    '''
                    print("stcQuater"),
                    print(self.stcQuater.q0),
                    print(self.stcQuater.q1),
                    print(self.stcQuater.q2),
                    print(self.stcQuater.q3)
                    '''
                else:
                    pass
                    #print("wrong data")

            elif self.__rxBuffer[1] == 0x5a:
                if self.__rxBuffer[10] == (sum(self.__rxBuffer) - self.__rxBuffer[10]) & 0xff :
                    #print("right data")   
                    self.stcSsn.sSVNum = float( self.ComplementToOriginal((self.__rxBuffer[3] << 8)|self.__rxBuffer[2] , 16) )
                    self.stcSsn.sPDOP = float( self.ComplementToOriginal((self.__rxBuffer[5] << 8)|self.__rxBuffer[4] , 16) ) / 10
                    self.stcSsn.sHDOP = float( self.ComplementToOriginal((self.__rxBuffer[7] << 8)|self.__rxBuffer[6] , 16) ) / 100
                    self.stcSsn.sVDOP = float( self.ComplementToOriginal((self.__rxBuffer[9] << 8)|self.__rxBuffer[8] , 16) ) / 100
                    '''
                    print("stcSsn"),
                    print(self.stcSsn.sSVNum),
                    print(self.stcSsn.sPDOP),                
                    print(self.stcSsn.sHDOP),
                    print(self.stcSsn.sVDOP)
                    '''
                else:
                    pass
                    #print("wrong data")

            else:
                print("解析错误")
            self.__key = 0

    def ComplementToOriginal(self, data, bits):
        #补码转原码，数据，位长
        #符号位
        signal = data & (1 << (bits - 1))
        
        if(signal > 0):
            "负数"
            absolute = data - signal
            compare = 0
            for i in range(0, bits - 1):
                compare += (1 << i)
                #print(bin(compare))
            #原码(有符号)
            original = ((absolute ^ compare) + 1) * -1
            return (original)
        else:
            "正数"
            #print(data)
            return data

'''
def main():
    imu = CJY901()
    a = 255
    print (imu.ComplementToOriginal(a, 8))

if __name__ == '__main__':
    main()
'''