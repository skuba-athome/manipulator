#!/usr/bin/env python
import math


# 32.5 is upper arm's length 29 is lower arm's length
def invKinematic(x, y, z):
    r = math.pow(29 * 29 - y * y, 0.5) + math.pow(10, -100)
    R = math.pow(x * x + z * z, 0.5)
    Zeta = math.acos((R * R - 32.5 * 32.5 - r * r) / (2 * 32.5 * r))
    ZetaII = math.asin(32.5 * math.sin(Zeta) / R)
    As20 = math.atan(x / abs(z)) - (Zeta - ZetaII)
    As21 = math.atan(y / (r * math.sin(Zeta)))
    Ae22 = (math.pi / 2) - math.acos(r * math.cos(Zeta) / 29.0)
    z2 = 32.5 * math.cos(As20) - z
    Ah40 = -(math.asin(y / math.pow(y * y + z2 * z2, 0.5)))
    Ah41 = math.acos((x - 32.5 * math.sin(As20)) / 29.0)
    Ah42 = -Ah40
    Ag43 = 0

    print "R", R
    print "r", r
    print "Zeta", math.degrees(Zeta)
    print "ZetaII", math.degrees(ZetaII)
    print "As20", math.degrees(As20)
    print "As21", math.degrees(As21)
    print "Ae22", math.degrees(Ae22)
    print "Ah40", math.degrees(Ah40)
    print "Ah41", math.degrees(Ah41)
    print "Ah42", math.degrees(Ah42)
    print "Ag43", math.degrees(Ag43)
    print "X is", (32.5 * math.sin(As20) + r * math.sin(math.pi - Zeta - As20))
    print "Y is", (29 * math.sin((math.pi / 2) - Ae22) * math.sin(As21))
    print "Z is", (r * math.cos(math.pi - Zeta - As20) - 32.5 * math.cos(As20))
    return [As20, As21, Ae22, Ah40, Ah41, Ah42, Ag43]


# if __name__ == "__main__":
    # invKinematic(0.521410161,-0.12,-0.103108891)
    # print "--"
    # invKinematic(0.503108891,-0.12,0.171410161)
    # print "--"
    # invKinematic(0.346410161,-0.12,-0.15)
    # print "--"
    # invKinematic(0.4,-0.12,-0.35)
    # print "--"
    # invKinematic(0.749999,-0.12,0)
    # print "--"
    # invKinematic(0.05,0.36,0.66)
    # print "--"
    # invKinematic(0.733218442309,-0.192329706124,-0.0967466984613)
    # print "--"


       