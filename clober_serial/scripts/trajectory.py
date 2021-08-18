#!/usr/bin/python3


import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class OdomType:
    def __init__(self):
        self.markerId = 0
        self.cnt = 0
        self.marker_pub = None

class OdomViz:
    def __init__(self):

        self.odomData = OdomType()
        self.odomData.marker_pub = rospy.Publisher("/marker/odom", Marker,queue_size=10)

        self.odomFiltered = OdomType()
        self.odomFiltered.marker_pub = rospy.Publisher("/marker/odom/filtered", Marker,queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odomCB)
        rospy.Subscriber("/odom/ekf/enc_imu", Odometry, self.odomFilteredCB)


    def odomCB(self, msg):
        if self.odomData.cnt > 100 :
            odom_x = msg.pose.pose.position.x
            odom_y = msg.pose.pose.position.y
            odom_yaw = self.getYaw(msg.pose.pose.orientation)
            self.draw(odom_x,odom_y,odom_yaw)
            self.odomData.cnt = 0
        else:
            self.odomData.cnt += 1


    def odomFilteredCB(self, msg):
        if self.odomFiltered.cnt > 0 :
            odom_x = msg.pose.pose.position.x
            odom_y = msg.pose.pose.position.y
            odom_yaw = self.getYaw(msg.pose.pose.orientation)
            self.draw2(odom_x,odom_y,odom_yaw)
            self.odomFiltered.cnt = 0
        else:
            self.odomFiltered.cnt += 1


    def getYaw(self,q):
        q_list = [q.x,q.y,q.z,q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        return yaw


    def draw(self,x,y,yaw):
        m = Marker()
        m.header.frame_id = "base_link"
        m.id = self.odomData.markerId
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
        self.odomData.marker_pub.publish(m)
        self.odomData.markerId +=1


    def draw2(self,x,y,yaw):
        m = Marker()
        m.header.frame_id = "base_link"
        m.id = self.odomFiltered.markerId
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
        m.color.r = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.1
        self.odomFiltered.marker_pub.publish(m)
        self.odomFiltered.markerId +=1

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
    