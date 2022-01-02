#!/usr/bin/env python
from typing import Set
import rospy
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import inf

DestinationX = 15
DestinationY = 7.5
Lidar_Forward = 0.0
Lidar_Right = 0.0
x = 0.0
y = 0.0
yaw = 0.0

def ListenLidar(inLaserScan):
    global Lidar_Forward
    global Lidar_Right
    # print(inLaserScan.ranges[0])
    Lidar_Forward = inLaserScan.ranges[0]
    Lidar_Right = inLaserScan.ranges[270]

def ListenOdom(inOdom):
    global x, y, yaw
    x = inOdom.pose.pose.position.x
    y = inOdom.pose.pose.position.y
    rot_q = inOdom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("MoveToGoal", anonymous=True)
Sub1 = rospy.Subscriber("/scan", LaserScan, ListenLidar)
Sub2 = rospy.Subscriber("/odom", Odometry, ListenOdom)
Pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

TurtleVelocity = Twist()
Speed_Forward = 0.3
Speed_Angular = 0.3
# Turtoise States = findangle, forward, turnleft, turnright
TurtoiseState = "turntodestination"
def SetState(inState):
    global TurtoiseState
    TurtoiseState = inState
    
def SetVelocity(inlinearX, inangularZ):
    for dec in range(4, 1, -1):
        #Linear X
        TurtleVelocity.linear.x = (inlinearX / dec)
        #Angular Z
        TurtleVelocity.angular.z = (inangularZ / dec)
        Pub1.publish(TurtleVelocity)

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    State = TurtoiseState

    if(State == "turntodestination"):
        DeltaAngle1 = atan2((DestinationY-y), (DestinationX-x))
        if(DeltaAngle1 - yaw > 0.0):
            SetVelocity(0.0, Speed_Angular)
        elif(DeltaAngle1 - yaw <= 0.0):
            SetState("forward")

    if(State == "forward"):
        SetVelocity(Speed_Forward, 0.0)
        if(Lidar_Forward < 0.5 and Lidar_Forward != inf):
            SetState("turnleft")

    if(State == "turnleft"):
        if(yaw <= 1.50):
            SetVelocity(0.0, Speed_Angular)
        #Forward
        elif(yaw > 1.50 and Lidar_Right != inf):
            SetVelocity(Speed_Forward, 0.0)
        elif(Lidar_Right == inf):
            rate.sleep()
            SetVelocity(Speed_Forward, 0.0)
            SetState("turnright")

    if(State == "turnright"):
        if(yaw >= 0.0):
            SetVelocity(0.0, Speed_Angular*-1)
        #Forward
        elif(yaw < 0.0):
            if(Lidar_Right == inf):
                SetVelocity(Speed_Forward, 0.0)
            elif(Lidar_Right != inf):
                SetState("turncorner")

    if(State == "turncorner"):
        if(Lidar_Right != inf):
            SetVelocity(Speed_Forward, 0.0)
        elif(Lidar_Right == inf):
            while(yaw > -1.50):
                SetVelocity(0.0, Speed_Angular*-1)
            SetVelocity(0.0, 0.0)
            SetState("forward_arctan2")

    if(State == "forward_arctan2"):
        DeltaAngle1 = atan2((DestinationY), (DestinationX))
        DeltaAngle2 = atan2((DestinationY-y), (DestinationX-x))
        if(DeltaAngle2 < DeltaAngle1):
            SetVelocity(Speed_Forward, 0.0)
        elif(DeltaAngle2 >= DeltaAngle1):
            SetVelocity(0.0, 0.0)
            if(DeltaAngle2 >= yaw):
                SetVelocity(0.0, Speed_Angular)
            else:
                SetVelocity(Speed_Forward, 0.0)
                if(x >= DestinationX and y >= DestinationY):
                    print("GOAL")
                    SetVelocity(0.0, 0.0)
                    break
                
    rate.sleep()