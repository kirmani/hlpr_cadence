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

class LookAtObject(Action):
	def __init__(self, object_name):
		self.object_name_ = object_name
		resource_name = 'object_' + object_name
		Action.__init__(self, 'look_at_' + resource_name, [resource_name],
                {resource_name: True},
                {resource_name: True})

	def Task(self):
		rospy.init_node('scan_scene')
		# Connect to the action client
		scan_client = actionlib.SimpleActionClient('lookat_waypoints_action_server', LookatWaypointsAction)
		rospy.logwarn("Waiting for scan scene server to load")
		scan_client.wait_for_server()
		rospy.logwarn("Scan scene loaded")

    #Generate some positions
    
		left_down = transform_helper(Vector3(1.0,-2.0,-1.0), 'pan_base_link')
		right_down = transform_helper(Vector3(1.0,2.0,-1.0), 'pan_base_link')
		center_down = transform_helper(Vector3(1.0,0.0,-1.0), 'pan_base_link')

    # Generate some times
		pause_2 = rospy.Duration(2.0)

    # Store them away to send off
		positions = [right_down,left_down,center_down]
		scan_times = [pause_2, pause_2, pause_2]
 
    # Send the goals

		goal = LookatWaypointsGoal()

		if(self.object_name_ == 'mug'):
			goal.scan_positions = [positions[0]]
			goal.scan_times = [scan_times[0]]
		elif(self.object_name_ == 'banana'):
			goal.scan_positions = [positions[1]]
			goal.scan_times = [scan_times[1]]
		elif(self.object_name_ == 'bowl'):
			goal.scan_positions = [positions[2]]
			goal.scan_times = [scan_times[2]]

		scan_client.send_goal(goal)

    # Print results
		rospy.loginfo("Waiting for scan to finish")
		scan_client.wait_for_result()
		rospy.loginfo(scan_client.get_goal_status_text())
		
		print("Looked at object: %s" % self.object_name_)

   
