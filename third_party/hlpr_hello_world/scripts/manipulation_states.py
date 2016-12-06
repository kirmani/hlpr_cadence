#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import sys
import time

from geometry_msgs.msg import Pose

from hlpr_speech_recognition.speech_listener import SpeechListener
from hlpr_speech_msgs.srv import SpeechService
#from hlpr_speech_synthesis import speech_synthesizer 
from hlpr_manipulation_utils import manipulator
from hlpr_manipulation_utils.manipulator import *
from hlpr_manipulation_utils.arm_moveit import *

from modules.utilities.baris_utils import *

_manipulator = None
_arm_planner = None
#_speech_synth = None
_isTheManipulationStateGlobalsInitialized = False

# see if it makes sense to pass these with userdata
def initGlobals():
  """
  Initialize manipulator, arm planner from hlpr_manipulation_utils
  Initialize speech synthesizer from hlr_speech_synthesis
  """
  print 'Initializing manipulation state globals'

  global _manipulator
  global _arm_planner
  global _speech_synth
  global _isTheManipulationStateGlobalsInitialized

  _arm_planner = ArmMoveIt()
  _manipulator = Manipulator()
  #_speech_synth = speech_synthesizer.SpeechSynthesizer()
  _isTheManipulationStateGlobalsInitialized = True






def sendPlan(arm, plannedTra):
  """
  Send the planned trajectory to the arm to follow
  """
  traj_goal = FollowJointTrajectoryGoal()
  traj_goal.trajectory = plannedTra.joint_trajectory
  arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
  arm.smooth_joint_trajectory_client.wait_for_result()   
  return arm.smooth_joint_trajectory_client.get_result() 






class ManipulateObjectMainState(smach.State):
  """
  Central state for the Manipulation Sub FSM
  """
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['moveArm', 'gripper', 'findObject', 'succeeded', 'failed', 'aborted','planPath', 'pickRetract'],
                         input_keys=['armGoalIn', 'graspResultIn', 'objectLocationIn', 'pathFoundIn', 'pickResultIn','targetPoseIn'], 
                         output_keys=['traOut','gripperCommandOut','statusOut'])
    # This keeps track of the current state
    self.status = 'begin'

    #  Load manipulator, arm planner and speech synthesizer as global variables
    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    #self.ss = _speech_synth


  def execute(self, userdata):
    """
    Define state transitions within manipulation sub FSM
    """
    userdata.statusOut = None
    rospy.loginfo('Manipulate object substate machine')

    if self.status is 'begin':
      self.status = 'findObject'
      return 'findObject'

    elif self.status is 'findObject':
      if userdata.objectLocationIn is None:
        print 'Failed because no object found'
        userdata.statusOut = 'failed'
        return 'failed'
      self.status = 'planning'
      return 'planPath'

    elif self.status is 'planning':
      if not userdata.pathFoundIn:
        print 'Failed because no plan found'
        userdata.statusOut = 'failed'
        return 'failed'
      self.status = 'preGrasp'
      return 'moveArm'

    elif self.status is 'preGrasp':
      if userdata.targetPoseIn is None:
        print 'Failed because could not execute'
        userdata.statusOut = 'failed'
        return 'failed'
      else:
        self.status = 'pickRetract'
        return 'pickRetract'

      #userdata.gripperCommandOut = 'close'
      #userdata.traOut = None
      #self.status = 'grasp'

    elif self.status is 'pickRetract':
      if userdata.pickResultIn:
        self.status = 'success'
        return 'succeeded'
      else:
        return 'failed'
    else:
      print 'How the hell did I manage to come here! Status: ' + str(self.status)
      #self.ss.say('Human I am not in a correct state, how did I get here?')
      time.sleep(1)
      return 'failed'  
  
    #elif self.status is 'grasp':
    #  if userdata.graspResultIn:
    #     userdata.gripperCommandOut = None
    #     userdata.traOut = 'retract'
    #     self.status = 'retract'
    #  else:
    #     userdata.gripperCommandOut = None
    #     userdata.traOut = 'retract'
    #     #self.status =           




class ManipulateObjectState(smach.State):
  """
  This state is not being used in the FSM
  """

  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','aborted'],
                         input_keys=['armGoalIn'])

  def execute(self, userdata):
    rospy.loginfo('Moving the arm')
    if userdata.armGoalIn is 'object':
        act = 'grasping'
    else:
        act = 'handing over'
    for i in range(0,11):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
      if i is 5:
        sys.stdout.write('- ' + act + ' -')
        sys.stdout.flush()
        time.sleep(2)
    print '> Good!'
    time.sleep(0.5)
    return 'succeeded'




class ExecuteTrajectoryState(smach.State):
  """
  Executes the trajectory provide by
  Plan Trajectory state
  """

  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['trajectoryIn'],
                         output_keys=['execResultOut'])

    # Initializing manipulation and speech  modules
    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.arm = _manipulator.arm # Arm()
    #self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()

    # Trajectory for person hardcoded as waypoints
    handOffTra = [[-1.90, 1.50, 0.50, -2.00, 3.00, 0.72],
                  [-1.80, 1.80, 1.00, -2.10, 2.50, 0.72],
                  [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90],
                  [-1.60, 2.20, 0.80, -2.20, 1.50, 1.20],
                  [-1.60, 2.40, 1.00, -2.50, 1.50, 1.20],
                  [-1.60, 2.60, 1.20, -3.14, 1.50, 1.20]]

    self.traDict = {'person':handOffTra}
    self.functionDict = {'retract':self.arm.upper_tuck}


  def execute(self, userdata):
    """
    Checks if the trajectory is for person, retract or towards object and executes
    Object Trajectory is passed fully planned
    Human Trajectory is hardcoded as waypoints and specified above
    Retract Trajectory is specified as a function call
    """

    rospy.loginfo('Moving the arm')
    userdata.execResultOut = None
    if userdata.trajectoryIn is None:
      print 'Received a None trajectory, this should not have happened'
      return 'failed'
    print 'Executing trajectory'
    #self.ss.say("Now executing the arm trajectory")

    if not isinstance(userdata.trajectoryIn, str):              # Check if Trajectory is to reach object
      if isinstance(userdata.trajectoryIn, list):
        self.arm.sendWaypointTrajectory(userdata.trajectoryIn)
      else:
        if len(userdata.trajectoryIn.joint_trajectory.points) < 1:
          return 'failed'
        sendPlan(self.arm, userdata.trajectoryIn)  
    else:
      try:                                                      # Check if Trajectory is for retract
        self.functionDict[userdata.trajectoryIn]()
      except KeyError:                                          # Check if Trajectory is for person
        try:
          self.arm.sendWaypointTrajectory(self.traDict[userdata.trajectoryIn])
        except KeyError:
          print 'Do not know how to execute ' + userdata.trajectoryIn
          return 'failed'
    userdata.execResultOut = 'done'
    return 'succeeded'





class UseGripperState(smach.State):
  """
  This gives the open/close command to gripper as per default setting from FSM or speech input
  """

  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['gripperCommandIn','waitForSpeech'],
                         output_keys = ['resultOut'])

    self.gripper_command = None

    # Initializing manipulation and speech modules
    self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM, None)
    if self.service_topic is None:
        rospy.logerr("Exiting: No speech topic given, is speech listener running?")
        exit()

    rospy.logwarn("Waiting for speech service")
    rospy.wait_for_service(self.service_topic)
    self.speech_service = rospy.ServiceProxy(self.service_topic, SpeechService)
    rospy.logwarn("Speech service loaded")

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.gripper = _manipulator.gripper


  def execute(self, userdata):
    """
    Passes the close/open command to gripper and outputs
    """

    rospy.loginfo('Using the gripper, how exciting!')

    # wait for speech input if flag set true
    # instead use the default gripper command
    if userdata.waitForSpeech:
      while not rospy.is_shutdown():
        try:                                                     # Fetch last speech command
          response = self.speech_service(True)
          self.last_command = response.speech_cmd
        except rospy.ServiceException:
          self.last_command = None

        if self.last_command == 'OPEN_HAND':
          self.gripper_command = 'open'
          break
        if self.last_command == 'CLOSE_HAND':
          self.gripper_command = 'close'
          break
        if self.last_command == 'END':
          return 'aborted'
        time.sleep(0.2)
    else:
      self.gripper_command = userdata.gripperCommandIn            # Fetch user data if no gripper speech command

    print 'Executing command ' + self.gripper_command
    if self.gripper_command == 'open':
      self.gripper.open()                                         # Execute open command
    elif self.gripper_command == 'close':
      self.gripper.close()                                        # Execute close command

    for i in range(0,2):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
    print '> Done!'
    userdata.resultOut = True
    time.sleep(0.5)
    return 'succeeded'




class PlanTrajectoryState(smach.State):
  """
  This state takes in the tranform from root to object
  and returns Planned trajectory, target pose to grab object and path found flag
  """

  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['objectLocationIn'],
                         output_keys=['traOut','pathFoundOut','targetPoseOut'])

    # Initialize manipulation and speech modules
    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    self.arm_planner = _arm_planner
    #self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()
    self.targetPose = None


  def execute(self, userdata):
    """
    Convert the object transform to a pose
    Use moveit arm planner to plan trajectory just above the object
    """

    rospy.loginfo('Calculating path')
    userdata.pathFoundOut = False
    userdata.traOut = None
    userdata.targetPoseOut = None

    print 'Creating the trajectory to pose ' + str(userdata.objectLocationIn)
    #self.ss.say('Creating the trajectory')# to pose ' + str(userdata.objectLocationIn))

    # Calculating target Pose for object
    # The target pose translation has offset in +ve z direction wrt to object pose
    # The target pose orientation is manually set to keep gripper facing down
    self.targetPose = transform2pose(userdata.objectLocationIn)
    if self.targetPose is None:
      print 'Received None pose, this should not have happened'
      #self.ss.say('Invalid pose received')
      return 'failed'
    #self.targetPose.position.z += 0.25
    self.targetPose.position.z += 0.12 # TODO: verify on the real robot (I think this takes into account the old EEF offset) which was 0.13 m
    self.targetPose.orientation = quatFromAngleAxis([-0.5094, 0.5094, 0.5094])

    # Pass target pose to moveit arm planner
    self.arm_planner.group[0].set_pose_reference_frame('base_link')
    plannedTra = self.arm_planner.plan_poseTargetInput(self.targetPose)
    #joints = self.arm_planner.get_IK(self.targetPose)
    #plannedTra = self.arm_planner.plan_jointTargetInput(joints)

    if plannedTra is None:
      print 'Could not find a plan'
      #self.ss.say('Could not find a plan')
      time.sleep(0.5)
      return 'failed'

    #self.ss.say("done!")
    time.sleep(0.5)
    userdata.traOut = plannedTra                             # Planned Trajectory output
    userdata.pathFoundOut = True                             # Flag to check if path found
    userdata.targetPoseOut = self.targetPose                 # Target Pose for planned trajectory
    return 'succeeded'




class CompositePickAndRetractState(smach.State):
  """
  This state grabs the object and retracts the arm
  """

  def __init__(self, ik_root = 'base_link'):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['initArmPoseIn'],
                         output_keys=['pickResultOut'])

    # Check for initialization of manipulation and speech utils
    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    # Pass global manipulation and speech utils as local variables
    #self.ss = _speech_synth
    self.manip = _manipulator
    self.arm_planner = _arm_planner
    self.ik_root = ik_root


  def execute(self, userdata):
    """
    Move the arm down, close the gripper and execute retract trajectory
    """

    rospy.loginfo('Picking and retracting')
    
    userdata.pickResultOut = False

    # Check for a recieved target pose
    if userdata.initArmPoseIn is None:
      print 'Received a None pose, this should not have happened'
      return 'failed'

    # Set target pose a fixed -ve offset in z-direction from current pose
    # Find straight-line EE trajectory from current to target pose
    curPose = rosPoseCopy(userdata.initArmPoseIn)
    targetPose = rosPoseCopy(userdata.initArmPoseIn)
    targetPose.position.z = max(0.91, targetPose.position.z-0.05)
    poses = straightLinePoses(curPose, targetPose)

    # Check if the EE trajectory satisfies the IK.
    # Return failure if not possible.
    # Execute trajectory if possible
    jntWps = self.arm_planner.wayPointIK(poses, 3, self.ik_root)
    if jntWps is None:
      print 'Could not calculate IK'
      #self.ss.say('Could not find inverse kinematics solution')
      return 'failed'
    self.manip.arm.sendWaypointTrajectory(jntWps)

    # Send gripper closing command
    time.sleep(0.5)
    #self.manip.gripper.close()
    #time.sleep(1.5)

    # Set target Pose fixed offset in +ve z-direction
    # Execute linear trajectory if possible
    curPose = rosPoseCopy(targetPose)
    targetPose.position.z += 0.12
    poses = straightLinePoses(curPose, targetPose)
    jntWps = self.arm_planner.wayPointIK(poses, 3, self.ik_root)
    if jntWps is None:
      print 'Could not calculate IK'
      #self.ss.say('Could not find inverse kinematics solution')
      #return 'failed'
    self.manip.arm.sendWaypointTrajectory(jntWps)

    # Retract the arm
    ut_wp = [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90]
    plannedTra = self.arm_planner.plan_jointTargetInput(ut_wp)
    if plannedTra is None:
      print 'Could not find a plan to retract the arm'
      #self.ss.say('Could not find a plan to retract the arm')
      time.sleep(0.5)
      return 'failed'
    sendPlan(self.manip.arm, plannedTra)
    self.manip.arm.upper_tuck()

    userdata.pickResultOut = True                  # Flag for pickup and retract
    return 'succeeded'

