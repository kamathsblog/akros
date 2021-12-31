#!/usr/bin/env python3

import rospy
import yaml
import os.path
import tf
import rospkg
from tf.transformations import euler_from_quaternion
from ds4_driver.msg import Status
from akros_msgs.msg import Mode
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point

class WaypointSequenceStore(object):
    def __init__(self, status_topic='status', mode_topic='mode', viz_topic='store_pose_array', viz_topic2='store_marker_array'):
        self._min_interval = 0.01
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._mode = Mode()
        self._pose_array = PoseArray()
        self._estimated_pose = [0, 0, 0]
        self._cursor = 0
        self._max_count = 0
        
        rospack = rospkg.RosPack()
        self._my_path = rospack.get_path('akros_behaviors')
        self._default_path = os.path.join(self._my_path, "config/waypoints_store.yaml")
        
        self._yaml_path = rospy.get_param('waypoint_store_yaml', self._default_path)
        self._localization_frame = rospy.get_param('localization_frame', 'map')
        self._base_frame = rospy.get_param('base_frame', 'base_link')
        
        self._listener = tf.TransformListener()
        
        # cleanup yaml file and load initial pose
        while not self._listener.waitForTransform(self._localization_frame, self._base_frame, rospy.Time(), rospy.Duration(60.0)): break
        (trans0,rot0) = self._listener.lookupTransform(self._localization_frame, self._base_frame, rospy.Time(0))
        empty_yaml = {'waypoints': {'waypoint0': {'x': trans0[0], 'y': trans0[1], 'theta': euler_from_quaternion(rot0)[2]}}}
        with open(self._yaml_path, 'w') as yamlfile:
                yaml.safe_dump(empty_yaml, yamlfile, sort_keys=False)
        rospy.loginfo("Waypoint Sequence Store initialized with YAML: %s", empty_yaml)
        
        self.append_pose_array(Point(*trans0), Quaternion(*rot0), 0)
        
        rospy.Subscriber(status_topic, Status, self.cb_status, queue_size=1)
        rospy.Subscriber(mode_topic, Mode, self.cb_mode, queue_size=1)
        self._pose_array_pub = rospy.Publisher(viz_topic, PoseArray, queue_size=1)
        
    def append_pose_array(self, position, orientation, cursor):
        pose = Pose()
        pose.position = position
        pose.orientation = orientation
        try:
            self._pose_array.poses.pop(cursor)
        except:
            pass
        self._pose_array.poses.insert(cursor, pose)

    def store_pose(self):
        new_dict = {
            'waypoint'+str(self._cursor) : {
                'x': self._estimated_pose[0],
                'y': self._estimated_pose[1],
                'theta': self._estimated_pose[2],
            }
        }

        with open(self._yaml_path, 'r') as yamlfile:
            cur_yaml = yaml.safe_load(yamlfile)
            cur_yaml['waypoints'].update(new_dict)
        
        if cur_yaml:
            with open(self._yaml_path, 'w') as yamlfile:
                yaml.safe_dump(cur_yaml, yamlfile, sort_keys=False)
            rospy.loginfo("Added waypoint%d (x: %f, y: %f, theta: %f)", self._cursor, self._estimated_pose[0], self._estimated_pose[1], self._estimated_pose[2])
        
    def cb_mode(self, msg):
        self._mode = msg

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval: return

        if self._mode.record:
            if msg.button_r3 and not self._prev.button_r3:
                self._cursor +=1
                if self._cursor > self._max_count:
                    self._max_count = self._cursor
                
                (trans,rot) = self._listener.lookupTransform(self._localization_frame, self._base_frame, rospy.Time(0))
                self._estimated_pose[0] = trans[0]
                self._estimated_pose[1] = trans[1]
                self._estimated_pose[2] = euler_from_quaternion(rot)[2]
                self.store_pose()
                
                # append pose_array and publish
                self.append_pose_array(Point(*trans), Quaternion(*rot), self._cursor)
                self._pose_array.header.frame_id = self._localization_frame
                self._pose_array.header.stamp = rospy.Time.now()
                self._pose_array_pub.publish(self._pose_array)
                
            if msg.button_r1 and not self._prev.button_r1:
                if 0 <= self._cursor < self._max_count:
                    self._cursor +=1
                rospy.loginfo("Cursor at %d", self._cursor+1)
                    
            if msg.button_r2 and not self._prev.button_r2:
                if 0 < self._cursor <= self._max_count :
                    self._cursor -=1
                rospy.loginfo("Cursor at %d", self._cursor+1)
           
        self._prev = msg
        self._last_pub_time = now

def main():
    try:
        rospy.init_node('wp_seq_store')
        WaypointSequenceStore()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("wp_seq_store node interrupted")

if __name__ == '__main__':
    main()
