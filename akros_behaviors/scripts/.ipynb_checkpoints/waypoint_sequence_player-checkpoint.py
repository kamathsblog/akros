#!/usr/bin/python

import rospy
import actionlib
import yaml
import os.path
import rospkg
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from akros_msgs.msg import Mode
from geometry_msgs.msg import Quaternion

class WaypointSequencePlayer(object):
    def __init__(self, mode_topic='mode'):
        self._done_once = False
        self._status    = 0
        self._goal_set  = False
        self._count     = -1 # initialized in error state
        self._elements  = 15
        self._mode      = Mode()
        self._goal      = MoveBaseGoal()
        self._goal.target_pose.header.frame_id = rospy.get_param('frame_id', 'map')
        
        rospack = rospkg.RosPack()
        self._my_path = rospack.get_path('akros_behaviors')
        self._default_path = os.path.join(self._my_path, "config/waypoints_store.yaml")
        
        self._yaml_path = rospy.get_param('waypoint_store_yaml', self._default_path)
        self._move_base = rospy.get_param('move_base_ns', '/move_base')
        
        rospy.Subscriber(mode_topic, Mode, self.cb_mode, queue_size=1)
        self._client = actionlib.SimpleActionClient(self._move_base, MoveBaseAction)
        
        self._client.wait_for_server()
        rospy.loginfo("Waypoint sequence player connected to move_base")
        self._status = 1 #set to 1 once connected to server and ready
        
    def cb_mode(self, msg):
        self._mode = msg
        
    def run(self):  
        while not rospy.is_shutdown():
            
            if not self._mode.play:
                if not self._done_once:
                    if self._goal_set:
                        self._client.cancel_goal()
                        self._goal_set = False
                    if self._count == -1:
                        self._count = 0
                    self._done_once = True
                    
            elif self._count != -1:
                with open(self._yaml_path, 'r') as yamlfile:
                    cur_yaml = yaml.safe_load(yamlfile)
                    
                self._elements = len(cur_yaml['waypoints'])
                
                if self._elements > 0:
                    if self._status < 4:
                        if self._count >= self._elements:
                            self._count = self._count % self._elements
                        
                        q = quaternion_from_euler(0, 0, cur_yaml['waypoints']['waypoint'+str(self._count)]['theta']) # this is a numpy array
                        self._goal.target_pose.pose.position.x = cur_yaml['waypoints']['waypoint'+str(self._count)]['x']
                        self._goal.target_pose.pose.position.y = cur_yaml['waypoints']['waypoint'+str(self._count)]['y']
                        self._goal.target_pose.pose.orientation = Quaternion(*q) # equivalent to Quaternion(q[0], q[1], q[2], q[3])

                        rospy.loginfo("Added waypoint%d (x: %f, y: %f, theta: %f)", 
                                      self._count, cur_yaml['waypoints']['waypoint'+str(self._count)]['x'], 
                                      cur_yaml['waypoints']['waypoint'+str(self._count)]['y'], 
                                      cur_yaml['waypoints']['waypoint'+str(self._count)]['theta'])
                        
                        self._client.send_goal(self._goal)
                        self._done_once = False
                        self._goal_set = True
                        
                        result = self._client.wait_for_result()
                        rospy.loginfo("Result for waypoint%d : %s", self._count, result)
                        self._status = self._client.get_state()
                        self._count += 1
                        
                    else:
                        if self._goal_set:
                            rospy.loginfo("Aborting due to critical/unknown error, stopping sequence")
                            self._client.cancel_goal()
                            self._goal_set = False
                        self._count = -1

def main():
    try:
        rospy.init_node('wp_seq_player')
        player = WaypointSequencePlayer()
        player.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("wp_seq_player node interrupted")
            
if __name__ == "__main__":
    main()