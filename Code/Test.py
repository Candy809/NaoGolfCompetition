#!/usr/bin/env python
# -*- coding:utf-8 -*-
# Author:Aileen
from naoqi import ALProxy
import almath
import time
import math
import motion
import thread
from VisualMoudle import *

def StiffnessOn(proxy):
    proxy.stiffnessInterpolation("Body", 1, 1)

def Grab():
    while True:
        RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
        if RighthandTouchedFlag == 1.0:
            print "right hand touched"
            # tts.say("给我一个高尔夫杆")
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4);
            time.sleep(4)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.4);
            time.sleep(3)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4);
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4);
            break

def HitBall(force = 0.35):
    if (alpha < 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR3, 0.4)
        time.sleep(3)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR4, force)
        time.sleep(2)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4)
    if (alpha > 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR4, 0.4)
        time.sleep(3)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR3, force)
        time.sleep(2)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4)
PORT = 9559
robotIP = "192.168.43.35"  # 此处输入机器人的IP地址
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
alpha = math.pi/2
PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
golfPositionJointAnglesR1 = [1.01402, 0.314159, 1.62907, 1.48035, -0.648924, 0.12]
golfPositionJointAnglesR2 = [1.02629, 0.314159, 1.62907, 1.48342, 0.230058, 0.12]
golfPositionJointAnglesR3 = [1.03549, 0.314159, 1.64747, 0.998676, 0.476658, 0.12]
golfPositionJointAnglesR4 = [1.03549, 0.314159, 1.66742, 0.971064, -0.980268, 0.12]
golfPositionJointAnglesR5 = [1.07998, 0.314159, 1.61986, 1.11679, 0.082794, 0.6]  # 松杆参数
golfPositionJointAnglesR6 = [1.07998, 0.314159, 1.61986, 1.11679, 0.082794, 0.12]  # 抓杆参数
StiffnessOn(motionPrx)
postureProxy.goToPosture("StandInit", 1.0)
Grab()
HitBall()