from manipulation.manipulator import *
from manipulation.arm_moveit import *
from modules.perception_module import ObjectSearch
from geometry_msgs.msg import *
from math import sqrt
import sys

from modules.utilities.baris_utils import *

def sendPlan(arm,plannedTra):
  traj_goal = FollowJointTrajectoryGoal()

  traj_goal.trajectory = plannedTra.joint_trajectory
  arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
  arm.smooth_joint_trajectory_client.wait_for_result()   
  return arm.smooth_joint_trajectory_client.get_result() 

if __name__ == '__main__':
  rospy.init_node('perception_moveit')

  manip = Manipulator()
  manip.gripper.open()
  
  #manip.linear_actuator.set_pos(0.6)
  #manip.arm.upper_tuck()   
  
  time.sleep(1)

  arm_planner = ArmMoveIt()

  needTuckingPre = False
  needTuckingPost = True

  objSearch = True

  ik_root = 'base_link'

  if needTuckingPre:
    ut_wp = [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90]
    plannedTra = arm_planner.plan_jointTargetInput(ut_wp)
    sendPlan(manip.arm, plannedTra)
    manip.arm.upper_tuck()

  if objSearch:
    os = ObjectSearch()

    if(os.objectSearch()):
      os.directLookAtObject()
    else:
      print 'no obj found'
      sys.exit()
      #os.pantilt.adjust([0,0],10,10)

    time.sleep(1.0)
    #targetPose = transform2pose(os.tf.getObjectTransform())
    #print os.tf.getObjectTransform()
    #print os.tf.getAverageObjectTransform(window_size=10,rate=10)
    targetPose = transform2pose(os.tf.getAverageObjectTransform(window_size=10,rate=10))

    #targetPose.position.y += 0.02
    targetPose.position.z += 0.25

  else:
    targetPose = Pose()
    targetPose.position.x =  1.16529724329
    targetPose.position.y =  0.156312971186
    targetPose.position.z =  1.06145966172

  #targetPose.orientation = quatFromAngleAxis([-0.5094,0.5094,0.5094])
  targetPose.orientation = quatFromAngleAxis([-0.49188,0.49188,0.49188])  


  #print os.tf.tr
  #print os.tf.tr_obj
  print targetPose


  plannedTra = arm_planner.plan_poseTargetInput(targetPose)
  #joints = arm_planner.get_IK(targetPose)
  #print joints
  #posesFromJ = arm_planner.get_FK(joints)
  #plannedTra = arm_planner.plan_jointTargetInput(joints)
  print 'You have 2 seconds before the trajectory is sent!'
  #time.sleep(2.0)
  sendPlan(manip.arm, plannedTra)

  
  curPose = rosPoseCopy(targetPose)
  targetPose.position.z = max(0.91, targetPose.position.z-0.12)
  poses = straightLinePoses(curPose, targetPose)
  jwps = arm_planner.wayPointIK(poses, 3, ik_root)
  #jwps = straightLineIK(arm_planner, curPose, targetPose)
  manip.arm.sendWaypointTrajectory(jwps)
  #plannedTra = arm_planner.plan_poseTargetInput(targetPose)
  #sendPlan(manip.arm, plannedTra)

  time.sleep(0.5)
  manip.gripper.close()
  time.sleep(1.5)

  curPose = rosPoseCopy(targetPose)
  targetPose.position.z += 0.15
  poses = straightLinePoses(curPose, targetPose)
  jwps = arm_planner.wayPointIK(poses, 3, ik_root)
  manip.arm.sendWaypointTrajectory(jwps)
  #manip.arm.sendWaypointTrajectory(straightLineIK(arm_planner, curPose, targetPose))
  #plannedTra = arm_planner.plan_poseTargetInput(targetPose)
  #sendPlan(manip.arm, plannedTra)

  if needTuckingPost:
    ut_wp = [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90]
    plannedTra = arm_planner.plan_jointTargetInput(ut_wp)
    sendPlan(manip.arm, plannedTra)
    manip.arm.upper_tuck()

  time.sleep(1.5)
  manip.gripper.open()

  

