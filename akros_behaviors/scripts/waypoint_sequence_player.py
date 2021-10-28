#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status

class WaypointSequencePlayer():
    def __init__(self):

        rospy.init_node('wp_seq_player')
        
        self._min_interval  = 0.01
        self._last_pub_time = rospy.Time()

        self.ros_sub_status   = rospy.Subscriber("status", Status, self.set_status, queue_size=1)

        self._prev           = Status()
        self._play           = False # paused on initialization
        self._count          = 0
        self._elements       = 0

        self._pub_mux = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    
    def set_status(self, msg):
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval: return
        
        # play/pause using triangle button
        if msg.button_triangle and not self._prev.button_triangle:
            self._play = not self._play

        self._prev = msg
        self._last_pub_time = now
        
    def run(self):  
        while not rospy.is_shutdown():
            
            if not self._play:
                rospy.loginfo("PAUSE")
                self._count = 0;
                # clear move_base
            else: # IF PLAY
                rospy.loginfo("PLAY")
                # read file and number of elements
                if self._elements == 0: # IF NO ELEMENTS, CLEAR EVERYTHING AND DO NOTHING
                    rospy.loginfo("NO ELEMENTS")
                    # clear move_base
                else: # IF ELEMENTS ARE PRESENT
                    if self._count >= self._elements: # CORRECT COUNTER VALUE
                        self._count = self._count % self._elements
                    # pass position[count] # PASS POSITION[COUNT] TO MOVE_BASE
                    # wait for server
                    # if succcess:
                    if True: #IF SUCCESS, INCREMENT COUNTER AND LOOP AGAIN
                        self._count = self._count + 1
                    # if normal failure
                    else if False: # IF NORMAL FAILURE, INCREMENT COUNTER AND PASS NEXT POSITION
                        self._count = self._count + 1
                        # pass position[count]
                        # wait for server:
                        #if success:
                        if True: # IF SECOND ATTEMPT IS SUCCESS, INCREMENT COUNTER AND LOOP AGAIN
                            self._count = self._count + 1
                        else: # IF SECOND ATTEMPT IS FAILURE (ANY KIND), CLEAR EVERYTHING AND DO NOTHING
                            self._count = 0 # CLEAR COUNTER, NEXT POSITION WILL BE START OF THE QUEUE
                            # clear move_base
                    # if critical failure
                    else if False: # IF CRITICAL FAILURE, CLEAR EVERYTHING AND DO NOTHING
                        self._count = 0 
                        # clear move_base
            
if __name__ == "__main__":
    mux = TwistMixer()
    mux.run()