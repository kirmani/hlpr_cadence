import actionlib
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from hlpr_lookat.msg import LookatWaypointsAction, LookatWaypointsGoal, LookatWaypointsResult

from action import Action
import subprocess
import time

def transform_helper(vec3, frame):
    pos = TransformStamped()
    pos.child_frame_id = frame
    pos.header = Header()
    pos.transform = Transform()
    pos.transform.translation = vec3

    return pos

class Nod(Action):
    def __init__(self):
        Action.__init__(self, 'look_center', [],
            {},
            {})

    def Task(self):
        rospy.init_node('scan_scene')
		# Connect to the action client
        scan_client = actionlib.SimpleActionClient('lookat_waypoints_action_server', LookatWaypointsAction)
        rospy.logwarn("Waiting for scan scene server to load")
        scan_client.wait_for_server()
        rospy.logwarn("Scan scene loaded")

        #Generate some positions
    
	
        center = transform_helper(Vector3(1.0,0.0,0.2), 'pan_base_link')
        down = transform_helper(Vector3(1.0,0.0,-2.0), 'pan_base_link')

        # Generate some times
        pause_2 = rospy.Duration(2.0)

        # Store them away to send off
        positions = [down, center]
        scan_times = [pause_2, pause_2]
 
        # Send the goals

        goal = LookatWaypointsGoal()
        goal.scan_positions = positions
        goal.scan_times = scan_times
		

        scan_client.send_goal(goal)

        # Print results
        rospy.loginfo("Waiting for scan to finish")
        scan_client.wait_for_result()
        rospy.loginfo(scan_client.get_goal_status_text())
		
        print("Looked center: %s")

   
