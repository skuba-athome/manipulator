#!/usr/bin/env python
import math


# 35 is upper arm's length 29 is lower arm's length
def invKinematic(x, y, z):
    #ForearmLength=29.0
    ForearmLength = 0.29
    #UpperarmLength=35.0
    UpperarmLength = 0.35
    FL=ForearmLength
    UL=UpperarmLength

    r = math.pow(FL * FL - y * y, 0.5) + 0.000000000000000001
    R = math.pow(x * x + z * z, 0.5)
    Zeta = math.acos((R * R - UL * UL - r * r) / (2 * UL * r))
    ZetaII = math.asin(UL * math.sin(Zeta) / R)
    As20 = math.asin(x / R) - (Zeta - ZetaII)
    elbowposition=[UL*math.sin(As20),UL*math.cos(As20)]
    As21 = math.atan(y / (r * math.sin(Zeta)))
    Ae22 = (math.pi / 2) - math.acos(r * math.cos(Zeta) / FL)
    z2 = UL * math.cos(As20) - z
    Ah40 = -(math.asin(y / math.pow(y * y + z2 * z2, 0.5)))
    Ah41 = math.acos((x - (UL * math.sin(As20))) / FL)
    Ah42 = -Ah40
    #Ag43 = math.pi/2]=9

    # print "R", R
    # print "r", r
    # print "Zeta", math.degrees(Zeta)
    # print "ZetaII", math.degrees(ZetaII)
    # print "As20", math.degrees(As20), As20
    # print "As21", math.degrees(As21), As21
    # print "Ae22", math.degrees(Ae22), Ae22
    # print "Ah40", math.degrees(Ah40)
    # print "Ah41", math.degrees(Ah41)
    # print "Ah42", math.degrees(Ah42)
    # #print "Ag43", math.degrees(Ag43)
    # print "X is", (UL * math.sin(As20) + r * math.sin(math.pi - Zeta - As20))
    # print "Y is", (FL * math.sin((math.pi / 2) - Ae22) * math.sin(As21))
    # print "Z is", (r * math.cos(math.pi - Zeta - As20) - UL * math.cos(As20))
    #print ( 35*math.sin(As20) + 29*math.cos( As20 - Ae22) )
    #print (-35*math.cos(As20) + 29*math.sin(As20 - Ae22))
    #return [As20, As21, Ae22, Ah40, Ah41, Ah42, Ag43]
    return [As20, As21, Ae22, Ah40, Ah41, Ah42]


       
