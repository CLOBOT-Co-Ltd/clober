#!/usr/bin/python3


import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


class OdomViz:
    def __init__(self):
        
        self.markerId = 0
        self.odomCnt = 0

        self.markerPub = rospy.Publisher("/marker/odom", Marker,queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odomCB)


    def odomCB(self, msg):
        if self.odomCnt > 100 :
            odom_x = msg.pose.pose.position.x
            odom_y = msg.pose.pose.position.y
            odom_yaw = self.getYaw(msg.pose.pose.orientation)
            self.draw(odom_x,odom_y,odom_yaw)
            self.odomCnt = 0
        else:
            self.odomCnt += 1


    def getYaw(self,q):
        q_list = [q.x,q.y,q.z,q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        return yaw

        
    def draw(self,x,y,yaw):
        m = Marker()
        m.header.frame_id = "base_link"
        m.id = self.markerId
        m.type = m.ARROW
        start = Point()
        start.x = x
        start.y = y
        m.points.append(start)
        end = Point()
        end.x = x + (0.1*math.cos(yaw))
        end.y = y + (0.1*math.sin(yaw))
        m.points.append(end)
        m.color.a = 1.0
        m.color.g = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.1
        self.markerPub.publish(m)

        self.markerId +=1


def main():
    try:
        rospy.init_node("odom_viz_node", anonymous=True)
        ov = OdomViz()
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")
        exit()

if __name__ == '__main__':
    main()
    