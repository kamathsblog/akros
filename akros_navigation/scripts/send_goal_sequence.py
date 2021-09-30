#! /usr/bin/env python

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

# temporary
# goals: x, y, rz, rw
# config 1
# goal_1 = [-3.63925839391002, 0.4033501890733998, -0.9999348335749032, 0.011416155374316856]
# goal_2 = [-2.8721106683544155, -0.5299627880775701, 0.9569901087542791, 0.2901205469222628]
# goal_3 = [-3.586190759439845, -0.47546525354437846, 0.018049162842433384, 0.9998371005922352]
# goal_4 = [-2.9905850342309304, 0.46641188278577533, 0.6971188841897575, 0.7169555504395146]
# goal_5 = [-3.586190759439845, -0.47546525354437846, 0.018049162842433384, 0.9998371005922352]

#config 2
goal_1 = [-2.8251769671262803, 0.3783810040573701, 0.3762924194089649, 0.9265009525604103]
goal_2 = [-2.863414564386513, -0.4077434296433166, 0.10746116414401784, 0.9942092828981293]
goal_3 = [-3.5703204206769916, 0.43134778345042846, -0.034251621105692784, 0.9994132410828026]
goal_4 = [-2.8251769671262803, 0.3783810040573701, 0.3762924194089649, 0.9265009525604103]
goal_5 = [-3.6148913083117855, -0.5025884448262337, -0.03888433094203508, 0.999243718422683]

sequence = (goal_1, goal_2, goal_3, goal_4, goal_5)
count = 1
status = 1
goal_set = False

def move_base_client():
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base server")
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    global count
    global status
    global sequence
    global goal_set
    
    for i in range(len(sequence)):
        
        count = i+1
        if (status < 4):
            goal.target_pose.pose.position.x = sequence[i][0]
            goal.target_pose.pose.position.y = sequence[i][1]
            goal.target_pose.pose.orientation.z = sequence[i][2]
            goal.target_pose.pose.orientation.w = sequence[i][3]
            
            client.send_goal(goal, active_cb=callback_active, feedback_cb=callback_feedback, done_cb=callback_done)
            goal_set = True
            client.wait_for_result()
            status = client.get_state()
            rospy.loginfo(status)
        else:
            rospy.loginfo("[%s] Sequence cancelled. Cancelling Goal %d..." % (status, i))
            if goal_set: client.cancel_goal()
            
        ## rospy.on_shutdown(client.stop())

def callback_active():
    rospy.loginfo("[%d] Goal %d accepted by move_base server. Processing..." % (status, count))

def callback_done(state, result):
    if state == 3:
        rospy.loginfo("[%s] Goal %d reached. Sending next goal..." % (str(state), count))
    elif state == 2:
        rospy.loginfo("[%s] Goal %d cancelled. Sending next goal..." % (str(state), count))
    elif state == 4:
        rospy.loginfo("[%s] Goal %d aborted. Cancelling sequence..." % (str(state), count))
    elif state > 4:
        rospy.loginfo("[%s] Goal %d done (unknown result). Result: %s. Cancelling sequence..." % (str(state), count, str(result)))
    global status
    status = state

def callback_feedback(feedback):
    # rospy.loginfo("Feedback:%s" % str(feedback))
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('send_goal_sequence')
        move_base_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Sequence interrupted before completion")