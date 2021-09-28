# Copyright 2021 CLOBOT Co. Ltd
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped,Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from rospkg.rospack import RosPack
from visualization_msgs.msg import Marker, MarkerArray
from clobot_msgs.msg import NavigationStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import json
import threading
import sys
import math

class TopologyGoal:
    def __init__(self, f_name):

        pack = RosPack()
        self.nodefile = pack.get_path("clober_tools")+"/topology/"+f_name+".json"
        self.topology = {"Map":"","Node":[],"Link":[]}
        self.pose = [0]*3

        self.cmdType = None
        self.goalNode = None
        self.goalList = None
        self.goalIdx = 0

        ## publisher ##
        self.goalPub = rospy.Publisher("/goal", PoseStamped, queue_size=1)

        self.nodePub = rospy.Publisher('/topology/nodes', MarkerArray, queue_size=1)
        self.linkPub = rospy.Publisher('/topology/links', MarkerArray, queue_size=1)

        ## subscriber ##
        rospy.Subscriber("/cmb/status", NavigationStatus, self.navCB)


    def sendGoal(self):
        if self.cmdType == "node":
            nodeId = self.goalNode

        elif self.cmdType == "path":
            if self.goalIdx >= len(self.goalList):
                print("path navigation finished")
                return
            nodeId = self.goalList[self.goalIdx]
        else:
            print("cmdType is not defined")
            return 

        goal = PoseStamped()

        value = self.findTopologyIdx(nodeId)
        
        if value == -1:
            print("node Id : ",nodeId," not exist")
        else:
            goal.pose.position.x = self.topology["Node"][value]["Position"]["x"]
            goal.pose.position.y = self.topology["Node"][value]["Position"]["y"]
            
            if self.cmdType == "node":     
                goal.pose.orientation.x = self.topology["Node"][value]["Orientation"]["x"]
                goal.pose.orientation.y = self.topology["Node"][value]["Orientation"]["y"]
                goal.pose.orientation.z = self.topology["Node"][value]["Orientation"]["z"]
                goal.pose.orientation.w = self.topology["Node"][value]["Orientation"]["w"]
    
            elif self.cmdType == "path":
                if self.goalIdx == len(self.goalList)-1:
                    goal.pose.orientation.x = self.topology["Node"][value]["Orientation"]["x"]
                    goal.pose.orientation.y = self.topology["Node"][value]["Orientation"]["y"]
                    goal.pose.orientation.z = self.topology["Node"][value]["Orientation"]["z"]
                    goal.pose.orientation.w = self.topology["Node"][value]["Orientation"]["w"]
                else:
                    next_goal_id = self.goalList[self.goalIdx+1]
                    next_goal_idx = self.findTopologyIdx(next_goal_id)
                    next_goal_x = self.topology["Node"][next_goal_idx]["Position"]["x"]
                    next_goal_y = self.topology["Node"][next_goal_idx]["Position"]["y"]

                    yaw_rad = math.atan2(next_goal_y-goal.pose.position.y, next_goal_x-goal.pose.position.x)
                    quat = self.getQuaternion(0.0,0.0,yaw_rad)
                    goal.pose.orientation.x = qaut[0]
                    goal.pose.orientation.y = qaut[1]
                    goal.pose.orientation.z = qaut[2]
                    goal.pose.orientation.w = qaut[3]
            else:
                pass

            self.goalPub.publish(goal)
            print("send goal Id : ",nodeId)
            return


    def findTopologyIdx(self,nodeId):
        for i in range(len(self.topology["Node"])):
            if self.topology["Node"][i]["ID"] == nodeId:
                return i
        return -1


    def getYaw(self,q):
        q_list = [q.x,q.y,q.z,q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        return yaw


    def getQuaternion(self,roll,pitch,yaw):
        quat = quaternion_from_euler(roll,pitch,yaw)
        return quat


    def navCB(self,msg):
        if msg.status == NavigationStatus.STATUS_REACHED:
            if self.cmdType == "path":
                print("send next goal")
                self.goalIdx += 1
                self.sendGoal()


    def checkId(self,nodeId):
        for i in range(len(self.topology["Node"])):
            if self.topology["Node"][i]["ID"] == nodeId:
                return True
        return False


    def getTopologyfromFile(self):
        try:
            print(self.nodefile)
            with open(self.nodefile, "rt") as f:
                self.topology = json.load(f)
                print(self.nodefile +" is already exist... use it!")
        except Exception as e:
            print('new file is created ', e)

    def setTopologyToFile(self):
        try:
            with open(self.nodefile, "wt") as f:
                json.dump(self.topology, f, indent=4)
        except Exception as e:
            print('error')


    def jsonload(self):
        self.getTopologyfromFile()

    def showNode(self):
        self.getTopologyfromFile()

        markerArray = MarkerArray()
        for i in range(len(self.topology["Node"])):

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = self.topology["Node"][i]["Position"]["x"]
            marker.pose.position.y = self.topology["Node"][i]["Position"]["y"]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            name_marker = Marker()
            name_marker.header.frame_id = 'map'
            name_marker.header.stamp = rospy.Time()
            name_marker.id = i+2000
            name_marker.type = Marker.TEXT_VIEW_FACING
            name_marker.scale.z = 0.3
            name_marker.text = self.topology["Node"][i]["ID"]
            name_marker.color.a = 1.0
            name_marker.color.r = 0.0
            name_marker.color.g = 0.0
            name_marker.color.b = 0.0
            name_marker.pose.position.x = self.topology["Node"][i]["Position"]["x"]+0.3
            name_marker.pose.position.y = self.topology["Node"][i]["Position"]["y"]+0.3

            markerArray.markers.append(marker)
            markerArray.markers.append(name_marker)

        self.nodePub.publish(markerArray)

    def showLink(self):
        self.getTopologyfromFile()
        markerArray = MarkerArray()

        for i in range(len(self.topology["Link"])):
            marker = Marker()

            s_node = self.topology["Link"][i]["Connected"][0]
            e_node = self.topology["Link"][i]["Connected"][1]
            print(s_node)
            print(e_node)

            start_point = Point()
            for j in range(len(self.topology["Node"])):
                if s_node == self.topology["Node"][j]["ID"] :
                    start_point.x = self.topology["Node"][j]["Position"]["x"]
                    start_point.y = self.topology["Node"][j]["Position"]["y"]
                    break

            end_point = Point()
            for j in range(len(self.topology["Node"])):
                if e_node == self.topology["Node"][j]["ID"] :
                    end_point.x = self.topology["Node"][j]["Position"]["x"]
                    end_point.y = self.topology["Node"][j]["Position"]["y"]
                    break

            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = marker.LINE_STRIP
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.points.append(start_point)
            marker.points.append(end_point)
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            name_marker = Marker()
            name_marker.header.frame_id = 'map'
            name_marker.header.stamp = rospy.Time()
            name_marker.id = i+2000
            name_marker.type = Marker.TEXT_VIEW_FACING
            name_marker.scale.z = 0.3
            name_marker.text = self.topology["Link"][i]["ID"]
            name_marker.color.a = 1.0
            name_marker.color.r = 0.0
            name_marker.color.g = 0.0
            name_marker.color.b = 0.0
            name_marker.pose.position.x = (start_point.x+end_point.x)/2 - 0.4
            name_marker.pose.position.y = (start_point.y+end_point.y)/2

            markerArray.markers.append(marker)
            markerArray.markers.append(name_marker)

        self.linkPub.publish(markerArray)



def main(filename):
    try:
        rospy.init_node("topology_goal_node", anonymous=True)
        writer = TopologyGoal(filename)

        t = threading.Thread(target=rospy.spin)
        t.setDaemon(True)
        t.start

        """ print usage """
        print("*********** Use key input ***********")
        print("q : quit")
        print("load : load file")
        print("node : set goal from topology nodes")
        print("path : set path from topology nodes")
        print("go : send goal by node or path")
        print("v : show nodes")
        print("*******************")

        while not rospy.is_shutdown():
            try:
                cmd = input("input : ")
                if cmd == 'q':
                    break
                elif cmd == 'node':
                    name = input("node name : ")
                    writer.cmdType = 'node'

                    if writer.checkId(name):
                        writer.goalNode = name
                        print("set goal node : ",name)

                elif cmd == 'path':
                    path = []
                    writer.cmdType = 'path'
                    writer.goalIdx = 0

                    while True:
                        name = input("path node name (press 'f' if finish) : ")
                                     
                        if writer.checkId(name):
                            path.append(name)
                            print("set goal path : ",path)
                   
                        if name == 'f':
                            break

                    writer.goalList = path

                elif cmd == 'go':
                    writer.sendGoal()
                elif cmd == 'load':
                    writer.jsonload()
                elif cmd == 'v':
                    writer.showNode()
                    writer.showLink()
            except ValueError:
                print("value error")

    except KeyboardInterrupt:
        print("shutting down")


if __name__ == "__main__":
    if len(sys.argv) != 2 :
        print("use... 'python topologyWriter.py filename'") 
        exit(0)
    main(sys.argv[1])
