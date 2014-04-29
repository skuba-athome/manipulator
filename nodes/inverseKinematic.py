#!/usr/bin/env python
from math import degrees,acos,atan,sqrt,sin,cos,asin,radians

gripper_length = 0.15

def invKinematic(x,y,z):
        print x,y,z
        a = sqrt((x*x)+(y*y))
        b = 0.12
        x_1 = degrees(acos(b/a))
        print x_1
        if(abs(y) < 0.01):
                theta_1 = 90 - x_1
        else:
                x_2 = degrees(atan(abs(x)/abs(y)))
                if(y > 0):
                        theta_1 = 180 - x_2 - x_1 
                else:
                        theta_1 = x_2 - x_1
        print 'Theta 1 :',theta_1
        x_1 = sin(radians(theta_1))*0.12
        y_1 = cos(radians(theta_1))*0.12*-1

        d = sqrt((x_1-x)**2+(y_1-y)**2)
        d -= gripper_length

        x_3 = d*cos(radians(theta_1)) + x_1
        y_3 = d*sin(radians(theta_1)) + y_1

        print 'Point extend :', x_3, y_3

        d = sqrt((x_1-x_3)**2+(y_1-y_3)**2+z**2)

        print 'Radius',d,x_1,x_1-x_3,y_1,y_1-y_3
        if(abs(z) < 0.00001):
                offset = 90
        else:
                offset = -1*degrees(atan(sqrt((x_1-x_3)**2+(y_1-y_3)**2)/z))
        r = 0.35
        R = 0.35
        x_2 = (d*d-r*r+R*R)/(2*d)
        print '---->',x_2,R
        print x_2,offset,degrees(acos(x_2/R))
        theta_2 = offset - degrees(acos(x_2/R))  
        print 'Theta 2 :',theta_2
        x_1 = d - x_2
        theta_3 = 90 - (offset-theta_2) + degrees(asin(x_1/r))
        print 'Theta 3 :',theta_3
        theta_4 = 90 + (theta_2) - theta_3
        print 'Theta 4 :',theta_4
        return [theta_1,theta_2,theta_3,theta_4]

if __name__ == "__main__":
        invKinematic(0.521410161,-0.12,-0.103108891)
        print "--"
        #invKinematic(0.503108891,-0.12,0.171410161)
        print "--"
        invKinematic(0.346410161,-0.12,-0.15)
        print "--"
        invKinematic(0.4,-0.12,-0.35)
        print "--"
        invKinematic(0.749999,-0.12,0)
        print "--"
        invKinematic(0.05,0.36,0.66)
        print "--"
        invKinematic(0.733218442309,-0.192329706124,-0.0967466984613)
        print "--"
