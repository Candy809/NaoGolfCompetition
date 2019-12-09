#!/usr/bin/env python
# -*- coding:utf-8 -*-
from naoqi import ALProxy
import almath
import time
import math
import motion
import thread
import sys


def StiffnessOn(proxy):
    proxy.stiffnessInterpolation("Body", 1, 1)


# def Grab():
#     while True:
#         RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
#         if RighthandTouchedFlag == 1.0:
#             print "right hand touched"
#             # tts.say("给我一个高尔夫杆")
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4);
#             time.sleep(4)
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.4);
#             time.sleep(3)
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4);
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4);
#             break


def HeadTouch():
    while True:
        headTouchedButtonFlag = memoryProxy.getData("FrontTactilTouched")
        if headTouchedButtonFlag == 1.0:
            print "front head touched"
            # tts.say("yes! i know!")
            break


def TouchBack():
    while True:
        # headif=memoryProxy.getData("MiddleTactilTouched")
        headif = memoryProxy.getData("RearTactilTouched")
        if headif == 1.0:
            global stop1
            stop1 = 1
            print "停止"
            sys.exit()
            # tts.say("头后传感器收到信号,我将停止")
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


def findball(angle, sub_angle=0):  # 返回红球的数据
    camProxy.setActiveCamera(1)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", angle * math.pi / 180, 0.8)  # math.pi 圆周率  180/圆周率就是弧度转换公式
    motionPrx.angleInterpolationWithSpeed("HeadPitch", sub_angle * math.pi / 180, 0.8)
    redballProxy.subscribe("redBallDetected")
    memoryProxy.insertData("redBallDetected", [])  # 将数据插入内存
    # 增加红球识别和取消红球识别
    time.sleep(2)
    for i in range(10):
        ballData = memoryProxy.getData("redBallDetected")

    # redballProxy.unsubscribe("redBallDetected")
    if (ballData != []):
        print("红球位置在")
        print(ballData)
        headangle = motionPrx.getAngles("HeadYaw", True)
        allballData = [headangle, ballData, 0]
        return allballData
    else:
        print("没找到")
        return []


def firstSearchredball():
    global _searchBallTimes, _stepStatue
    _searchBallTimes += 1
    camProxy.setActiveCamera(1)

    if _searchBallTimes <= 3:
        searchRange = -60
    else:
        searchRange = -120

    search = searchRange
    while search <= -searchRange:
        findballinfo = findball(search)
        if findballinfo != []:
            return findballinfo
        else:
            search += 60
    findballinfo = findball(search)
    print("没有红球")
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.25, 0.0, 0.0, _stepStatue)  # 寻找距离修正 向前走
    allballDatatest = [0, 0, 1]
    return allballDatatest

# def firstSearchredball2():
#     global _searchBallTimes, _stepStatue
#     _searchBallTimes += 1
#     camProxy.setActiveCamera(1)
#
#     if _searchBallTimes <= 3:
#         searchRange = -60
#     else:
#         searchRange = -120
#
#     search = searchRange
#     while search <= -searchRange:
#         findballinfo = findball(angle=search, sub_angle=15)
#         if findballinfo != []:
#             return findballinfo
#         else:
#             search += 60
#     findballinfo = findball(search)
#     print("没有红球")
#     motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
#     motionPrx.setMoveArmsEnabled(False, False)
#     motionPrx.moveTo(-0.15, 0.0, 0.0, _stepStatue)  # 寻找距离修正 向前走
#     allballDatatest = [0, 0, 1]
#     return allballDatatest

def CalculateRobotToRedball(allballData):
    h = 0.478
    # 机器人行走参数
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.4
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    headangle = allballData[0]  # 头偏转角
    wzCamera = allballData[1][1][0]  # alpha角
    wyCamera = allballData[1][1][1]  # beta角
    isenabled = False
    x = 0.0
    y = 0.0
    theta = headangle[0] + wzCamera

    motionPrx.setMoveArmsEnabled(False, False)
    # 接下来，第一次，机器人转到正对红球的方向
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (39.7 * math.pi / 180.0)
    x = h / (math.tan(thetav)) - 0.3
    theta = 0.0
    motionPrx.setMoveArmsEnabled(False, False)
    # 接下来，第二次，机器人走到距离红球20厘米的位置
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    motionPrx.waitUntilMoveIsFinished()
    effectornamelist = ["HeadPitch"]
    timelist = [0.5]
    targetlist = [30 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, isenabled)
    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    # 接下来，第三次，机器人修正角度对准红球
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = (h - 0.1) / (math.tan(thetav)) - 0.07
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    # 接下来，第四次，机器人走到距离红球10厘米的位置
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    dx = (h - 0.1) / (math.tan(thetav))  # dx作为了三角形的一条边
    return dx


def firstSearchNAOmark():
    headYawAngle = -2.0
    camProxy.setActiveCamera(0)
    currentCamera = "CameraTop"

    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    landmarkProxy.subscribe("landmarkTest")
    markData = memoryProxy.getData("LandmarkDetected")
    while (headYawAngle < 2.0):
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.1)
        time.sleep(1)
        for i in range(10):
            markData = memoryProxy.getData("LandmarkDetected")
            if (markData and isinstance(markData, list) and len(markData) >= 2):
                break

        if (markData and isinstance(markData, list) and len(markData) >= 2):
            tts.say("i saw landmark!")
            landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
            # Retrieve landmark center position in radians.
            markwzCamera = markData[1][0][0][1]
            markwyCamera = markData[1][0][0][2]
            # Retrieve landmark angular size in radians.
            markangularSize = markData[1][0][0][3]
            print markwzCamera
            print markwyCamera
            print markangularSize
            headangle = motionPrx.getAngles("HeadYaw", True);
            print headangle
            markheadangle = markwzCamera + headangle[0]
            allmarkdata = [markwzCamera, markwyCamera, markangularSize, markheadangle, landmarkFlag]
            print allmarkdata
            return allmarkdata

            break
        else:
            tts.say("where is landmark ?")
            markwzCamera = 0
            markwyCamera = 0
            markangularSize = 0

        headYawAngle = headYawAngle + 0.5
    tts.say("i can not find landmark ! I will hit the ball directly ! ")  # 后面可以加击球程序
    print "landmark is not in sight !"
    landmarkFlag = 1  # landmark识别符0代表识别到，1代表未识别到。
    allmarkdata = [0, 0, 0, 0, landmarkFlag]
    return allmarkdata
    # landmarkProxy.unsubscribe("landmarkTest")

def robotTolandmark(allmarkdata):
    currentCamera = "CameraTop"
    landmarkTheoreticalSize = 0.105  # in meters
    wzCamera = allmarkdata[0]
    wyCamera = allmarkdata[1]
    angularSize = allmarkdata[2]
    # 机器人与红球xdistance ，机器人与mark之间sdistance ， 两条线的夹角为angle
    # 计算距离
    distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))

    # 获取当前机器人到摄像头的距离的变换矩阵
    transform = motionPrx.getTransform(currentCamera, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)

    # 计算指向NAOmark的旋转矩阵
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

    # 摄像头到NAOmark的矩阵
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

    # 机器人到NAOmark的矩阵
    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
    x = robotToLandmark.r1_c4
    y = robotToLandmark.r2_c4
    z = robotToLandmark.r3_c4
    sdistance = math.sqrt(x * x + y * y)  # 机器人与mark的距离sdistance
    angle = math.atan(y/x)
    robotTolandmarkdata = [angle, sdistance, x, y, z]
    print "x " + str(robotToLandmark.r1_c4) + " (in meters)"
    print "y " + str(robotToLandmark.r2_c4) + " (in meters)"
    print "z " + str(robotToLandmark.r3_c4) + " (in meters)"
    return robotTolandmarkdata

def TriangleCalculation(x, s, alpha):
    if (alpha < 0.0):
        alpha = abs(alpha)
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
        l = math.sqrt(l2)
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print("theta"+str(theta))
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            print(theta2)
            turnAngle1 = theta2 - 0.04
            print(turnAngle1)
            dis1 = 0 - (x * math.sin(theta2))# 修正值0.0
            turnAngle2 = 0
            dis2 = x * math.cos(theta2) - 0.05

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            print(theta2)
            turnAngle1 = 0 - theta2 + 0.4
            print(turnAngle1)
            dis1 = x * math.cos(theta) + 0.08  # 修正值
            turnAngle2 = 0
            dis2 = x * math.sin(theta) - 0.08

        turndata = [dis1, dis2, turnAngle1, turnAngle2]
        return turndata
    if (alpha > 0.0):
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
        l = math.sqrt(l2)
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print(theta)
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            print(theta2)
            turnAngle1 = 0 - theta2 + 0.4
            # turnAngle1 = -math.pi + theta2
            print(turnAngle1)
            dis1 = x * math.sin(theta2) + 0.10  # 修正值
            # turnAngle2 = -math.pi-turnAngle1
            turnAngle2 = 0
            dis2 = x * math.cos(theta2) - 0.12

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            print(theta2)
            turnAngle1 = theta2 - 0.4
            print(turnAngle1)
            dis1 = -x * math.cos(theta) - 0.08
            # turnAngle2 = math.pi-turnAngle1
            turnAngle2 = 0
            dis2 = x * math.sin(theta) - 0.09

        turndata = [dis1, dis2, turnAngle1, turnAngle2]
        return turndata

def TriangleCalculation2(x, s, alpha):
    if (alpha < 0.0):
        alpha = abs(alpha)
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
        l = math.sqrt(l2)
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print("theta"+str(theta))
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            print(theta2)
            turnAngle1 = theta2 - 0.20
            print(turnAngle1)
            dis1 = 0 - (x * math.sin(theta2))# 修正值0.0
            turnAngle2 = 0
            dis2 = x * math.cos(theta2) - 0.05

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            print(theta2)
            turnAngle1 = 0 - theta2 + 0.4
            print(turnAngle1)
            dis1 = x * math.cos(theta) + 0.08  # 修正值
            turnAngle2 = 0
            dis2 = x * math.sin(theta) - 0.08

        turndata = [dis1, dis2, turnAngle1, turnAngle2]
        return turndata
    if (alpha > 0.0):
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
        l = math.sqrt(l2)
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print(theta)
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            print(theta2)
            turnAngle1 = 0 - theta2 + 0.4
            # turnAngle1 = -math.pi + theta2
            print(turnAngle1)
            dis1 = x * math.sin(theta2) + 0.18  # 修正值
            # turnAngle2 = -math.pi-turnAngle1
            turnAngle2 = 0
            dis2 = x * math.cos(theta2) - 0.16

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            print(theta2)
            turnAngle1 = theta2 - 0.4
            print(turnAngle1)
            dis1 = -x * math.cos(theta) - 0.08
            # turnAngle2 = math.pi-turnAngle1
            turnAngle2 = 0
            dis2 = x * math.sin(theta) - 0.09

        turndata = [dis1, dis2, turnAngle1, turnAngle2]
        return turndata

# def TriangleCalculation2(x, s, alpha):
#     if (alpha < 0.0):
#         alpha = abs(alpha)
#         l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
#         l = math.sqrt(l2)
#         costheta = (x * x + l2 - s * s) / (2 * x * l)
#         theta = math.acos(costheta)
#         if theta >= math.pi / 2:
#             theta2 = theta - math.pi / 2
#             turnAngle1 = theta2
#             dis1 = 0 - (x * math.sin(theta2) + 0.05)  # 修正值0.0
#             turnAngle2 = -1.91/180 *math.pi
#             dis2 = x * math.cos(theta2) - 0.03
#
#         if theta < math.pi / 2:
#             theta2 = math.pi / 2 - theta
#             turnAngle1 = 0 - theta2
#             dis1 = x * math.cos(theta) + 0.1  # 修正值
#             turnAngle2 = 0
#             dis2 = x * math.sin(theta) - 0.09
#
#         turndata = [dis1, dis2, turnAngle1, turnAngle2]
#         return turndata
#     if (alpha > 0.0):
#         l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
#         l = math.sqrt(l2)
#         costheta = (x * x + l2 - s * s) / (2 * x * l)
#         theta = math.acos(costheta)
#         if theta >= math.pi / 2:
#             theta2 = theta - math.pi / 2
#             turnAngle1 = 0 - theta2
#             dis1 = x * math.sin(theta2) + 0.05  # 修正值
#             # turnAngle2 = -math.pi-turnAngle1
#             turnAngle2 = -1.91/180 *math.pi
#             dis2 = x * math.cos(theta2) - 0.11
#
#         if theta < math.pi / 2:
#             theta2 = math.pi / 2 - theta
#             turnAngle1 = theta2
#             dis1 = -x * math.cos(theta) - 0.02
#             # turnAngle2 = math.pi-turnAngle1
#             turnAngle2 = 0
#             dis2 = x * math.sin(theta) - 0.14
#
#         turndata = [dis1, dis2, turnAngle1, turnAngle2]
#         return turndata

def AdjustPosition(turndata):
    dis1 = turndata[0]
    dis2 = turndata[1]
    turnAngle1 = turndata[2]
    turnAngle2 = turndata[3]
    # 机器人行走参数
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.4
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, 0.0, turnAngle1,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(dis2, 0.0, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, 0.0, turnAngle2,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, dis1, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

def TurnAfterHitball():
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # 机器人行走参数
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.4
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0
    if (alpha < 0.0):
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.0, 0.0, -math.pi / 2,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.4, 0.0, 0.0,  # 此处参数可调，设置搜寻距离
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])

    if (alpha > 0.0):
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.0, 0.0, math.pi / 2,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.4, 0.0, 0.0,  # 此处参数可调
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])


def Close():  # 松杆
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("HeadYaw")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LAnklePitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.349794, -0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LElbowRoll")
    times.append([1, 2, 3, 4])
    keys.append([-0.98632, -0.98632, -0.98632, -0.98632])

    names.append("LElbowYaw")
    times.append([1, 2, 3, 4])
    keys.append([-1.37757, -1.37757, -1.37757, -1.37757])

    names.append("LHand")
    times.append([1, 2, 3, 4])
    keys.append([0.2572, 0.2572, 0.2572, 0.2572])

    names.append("LHipPitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.450955, -0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LKneePitch")
    times.append([1, 2, 3, 4])
    keys.append([0.699462, 0.699462, 0.699462, 0.699462])

    names.append("LShoulderPitch")
    times.append([1, 2, 3, 4])
    keys.append([1.43885, 1.43885, 1.43885, 1.43885])

    names.append("LShoulderRoll")
    times.append([1, 2, 3, 4])
    keys.append([0.268407, 0.268407, 0.268407, 0.268407])

    names.append("LWristYaw")
    times.append([1, 2, 3, 4])
    keys.append([-0.016916, -0.016916, -0.016916, -0.016916])

    names.append("RAnklePitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.354312, -0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RElbowRoll")
    times.append([1, 2, 3, 4])
    keys.append([0.958791, 0.958791, 0.958791, 0.046062])

    names.append("RElbowYaw")
    times.append([1, 2, 3, 4])
    keys.append([1.67355, 1.67355, 1.21949, 1.19955])

    names.append("RHand")
    times.append([1, 2, 3, 4])
    keys.append([0.2216, 0.2216, 0.2216, 0.2216])

    names.append("RHipPitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.451038, -0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RKneePitch")
    times.append([1, 2, 3, 4])
    keys.append([0.699545, 0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")
    times.append([1, 2, 3, 4, 5.2])
    keys.append([1.03856, 0.412688, 0.412688, 1.44967, 1.48528])

    names.append("RShoulderRoll")
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.265341, 0.294486, -0.285367, -0.963394, -0.349066])

    names.append("RWristYaw")
    times.append([1, 2, 3, 4])
    keys.append([-0.955723, -0.914306, -0.937315, 0.460158])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)


def Raise():  # 抬杆
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("HeadYaw")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("LAnklePitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("LElbowRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.98632, -0.98632, -0.98632])

    names.append("LElbowYaw")
    times.append([1.6, 2.84, 4.12])
    keys.append([-1.37757, -1.37757, -1.37757])

    names.append("LHand")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.2572, 0.2572, 0.2572])

    names.append("LHipPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("LHipYawPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("LKneePitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.699462, 0.699462, 0.699462])

    names.append("LShoulderPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([1.43885, 1.43885, 1.43885])

    names.append("LShoulderRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.268407, 0.268407, 0.268407])

    names.append("LWristYaw")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.016916, -0.016916, -0.016916])

    names.append("RAnklePitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("RElbowRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.046062, 0.958791, 0.958791])

    names.append("RElbowYaw")
    times.append([1.6, 2.84, 4.12])
    keys.append([1.19955, 1.21949, 1.67355])

    names.append("RHand")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.2216, 0.2216, 0.2216])

    names.append("RHipPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([0, 0, 0])

    names.append("RKneePitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")
    times.append([1.6, 2.84, 4.12])
    keys.append([1.44967, 0.412688, 0.412688])

    names.append("RShoulderRoll")
    times.append([1.6, 2.84, 4.12])
    keys.append([-0.963394, -0.285367, 0.294486])

    names.append("RWristYaw")
    times.append([1.6, 2.84, 4.12])
    keys.append([0.460158, -0.937315, -0.914306])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

PORT = 9559
robotIP = "127.0.0.1"  # 此处输入机器人的IP地址

stop1 = 0
landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
redballFlag = 0  # redball识别符0代表识别到，1代表未识别到。
# 机器人行走参数
maxstepx = 0.04
maxstepy = 0.14
maxsteptheta = 0.4
maxstepfrequency = 0.6
stepheight = 0.02
torsowx = 0.0
torsowy = 0.0
_searchBallTimes = 0
_stepStatue = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
               ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
_stepStatue2 = [["MaxStepX", 0.02], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
                ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
tts = ALProxy("ALTextToSpeech", robotIP, PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
landmarkProxy = ALProxy("ALLandMarkDetection", robotIP, PORT)

PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
golfPositionJointAnglesR1 = [1.01402, 0.314159, 1.62907, 1.48035, -0.648924, 0.12]
golfPositionJointAnglesR2 = [1.02629, 0.314159, 1.62907, 1.48342, 0.230058, 0.12]
golfPositionJointAnglesR3 = [1.03549, 0.314159, 1.64747, 0.998676, 0.476658, 0.12]
golfPositionJointAnglesR4 = [1.03549, 0.314159, 1.66742, 0.971064, -0.980268, 0.12]
golfPositionJointAnglesR5 = [1.07998, 0.314159, 1.61986, 1.11679, 0.082794, 0.6]  # 松杆参数
golfPositionJointAnglesR6 = [1.07998, 0.314159, 1.61986, 1.11679, 0.082794, 0.12]  # 抓杆参数
# ------------------------------------------------------------------------------------------------------------#
StiffnessOn(motionPrx)
postureProxy.goToPosture("StandInit", 1.0)
thread.start_new(TouchBack, ())
# -------------------------------------------------
alpha = -math.pi / 2  # 初始值
HitBall()  # 击球
HeadTouch()  # 触摸前额开始
time.sleep(1)
Close()
while True:
    allballData = firstSearchredball()  # 红球定位 返回红球位置信息
    if (allballData[2] == 0):
        break
x = CalculateRobotToRedball(allballData)  # 计算 机器人与红球的距离并走到距离红球0.1米左右
s = 1.98  # 获取机器人到naomark距离
alpha = 0.123  # 获取 三角形夹角
turndata = TriangleCalculation2(x, s, alpha)  # 计算旋转数据
AdjustPosition(turndata)  # 调整机器人位置
motionPrx.moveTo(0.0, 0.0, 0.0, _stepStatue)
Raise()
HitBall(1.0)  # 击球
Close()
TurnAfterHitball()  # 击球后旋转
while True:
    allballData = firstSearchredball()  # 红球定位 返回红球位置信息
    if (allballData[2] == 0):
        break
x = CalculateRobotToRedball(allballData)  # 计算 机器人与红球的距离并走到距离红球0.1米左右
s = 2.98  # 获取机器人到naomark距离
alpha = -0.72  # 获取 三角形夹角
turndata = TriangleCalculation2(x, s, alpha)  # 计算旋转数据
AdjustPosition(turndata)  # 调整机器人位置
motionPrx.moveTo(0.0, 0.0, 0.0, _stepStatue)
Raise()
HitBall(1.0)  # 击球
Close()
motionPrx.moveTo(0.0, 0.0, -0.8, _stepStatue)
while True:
    while True:
        allballData = firstSearchredball()  # 红球定位 返回红球位置信息
        if (allballData[2] == 0):
            break

    x = CalculateRobotToRedball(allballData)  # 计算 机器人与红球的距离并走到距离红球0.1米左右
    allmarkdata = firstSearchNAOmark()  # NAOmark定位
    if (allmarkdata[4] == 0):
        robotTolandmarkdata = robotTolandmark(allmarkdata)  # 返回夹角angle和机器人与mark距离
        s = robotTolandmarkdata[1]  # 获取机器人到naomark距离
        alpha = robotTolandmarkdata[0]  # 获取 三角形夹角
        turndata = TriangleCalculation2(x, s, alpha)  # 计算旋转数据
        AdjustPosition(turndata)# 调整机器人位置



        motionPrx.moveTo(0.0, 0.0, 0.0, _stepStatue)
        # keepdis(robotIP)
        # h = 0.478
        # isenabled = False
        # motionPrx.waitUntilMoveIsFinished()
        # effectornamelist = ["HeadPitch"]
        # timelist = [0.5]
        # targetlist = [30 * math.pi / 180.0]
        # motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, isenabled)
        # time.sleep(1.5)
        # val = memoryProxy.getData("redBallDetected")
        # ballinfo = val[1]
        # thetah = ballinfo[0]
        # thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
        # dx = (h - 0.1) / (math.tan(thetav))  # dx作为了三角形的一条边
        # allmarkdata = firstSearchNAOmark()  # NAOmark定位
        # if (allmarkdata[4] == 0):
        #     robotTolandmarkdata = robotTolandmark(allmarkdata)  # 返回夹角angle和机器人与mark距离
        #     s = robotTolandmarkdata[1]  # 获取机器人到naomark距离
        #     alpha = robotTolandmarkdata[0]  # 获取 三角形夹角
        #     turndata = TriangleCalculation(dx, s, alpha)  # 计算旋转数据
        #     AdjustPosition(turndata)
        Raise()
        HitBall()  # 击球
        Close()
        TurnAfterHitball()  # 击球后旋转

    while allmarkdata[4] == 1:
        motionPrx.moveTo(0.0, 0.0, -math.pi/2, _stepStatue)
        allmarkdata = firstSearchNAOmark()
        if (allmarkdata[4] == 0):
            motionPrx.moveTo(0.0, -0.3, 0, _stepStatue)
            break
        else:
            motionPrx.moveTo(0.0, 0.0, -math.pi/2, _stepStatue)
            continue
