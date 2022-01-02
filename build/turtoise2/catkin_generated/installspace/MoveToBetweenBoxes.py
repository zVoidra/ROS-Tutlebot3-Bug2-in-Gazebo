#!/usr/bin/env python3
import rospy
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

DestinationX = 4.0
DestinationY = 4.0
x = 0.0
y = 0.0
yaw = 0.0

def OdometryMessage(inOdom):
    global x 
    global y 
    global yaw
    x = inOdom.pose.pose.position.x
    y = inOdom.pose.pose.position.y

    rot_q = inOdom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("MoveToBetweenBoxes", anonymous=True)

sub1 = rospy.Subscriber("/odom", Odometry, OdometryMessage)
pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

TurtleVelocity = Twist()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    DeltaAngle1 = atan2(DestinationY, DestinationX)
    if(DeltaAngle1 - yaw) > 0.1:
        TurtleVelocity.linear.x = 0.0
        TurtleVelocity.angular.z = 0.3
    else:
        TurtleVelocity.linear.x = 0.5
        TurtleVelocity.angular.z = 0.0
    pub1.publish(TurtleVelocity)
    if(x >= 4 and y >= 3):
        TurtleVelocity.linear.x = 0.0
        break
    print("Pos: "+" x:"+str(x)+" y:"+str(y))
    rate.sleep()

# PubMovement = rospy.Publisher('cmd/vel', Twist, queue_size=10)
