#!/usr/bin/env python
import rospy
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

DestinationX = 3.0
DestinationY = 4.0
x = 0.0
y = 0.0
yaw = 0.0

def OdometryMessage(inOdom):
    global x, y, yaw
    x = inOdom.pose.pose.position.x
    y = inOdom.pose.pose.position.y

    rot_q = inOdom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("MoveToBetweenBoxes", anonymous=True)

sub1 = rospy.Subscriber("/odom", Odometry, OdometryMessage)
pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


TurtleVelocity = Twist()

def SetVelocity(inlinearX, inangularZ):
    TurtleVelocity.linear.x = inlinearX
    TurtleVelocity.angular.z = inangularZ
    pub1.publish(TurtleVelocity)

def SetYaw0():
    print("Yaw: "+str(yaw))
    if(yaw > 0):
        SetVelocity(0.0, -0.2)
        return False
    else:
        SetVelocity(0, 0)
        return True
    

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    DeltaAngle1 = atan2(DestinationY, DestinationX)

    #Hedef
    if(x >= DestinationX and y >= DestinationY):
        SetVelocity(0, 0)
        if(SetYaw0()):
            SetVelocity(0, 0)
            break

    elif(DeltaAngle1 - yaw) > 0.2:
        SetVelocity(0.0, 0.1)
    else:
        SetVelocity(0.2, 0.0)
    
    rate.sleep()
