#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import sys
import time

from hlpr_speech_recognition.speech_listener import SpeechListener
from hlpr_speech_msgs.srv import SpeechService
from perception_states import *
from navigation_states import *
from manipulation_states import *
from hlpr_speech_synthesis import speech_synthesizer
from hlpr_manipulation_utils.manipulator import *

#comments:
#execute might take all the cycles, not very c6 like, see if smach has an updateOnce/executeOnce option

class IdleState(smach.State):
  """
  Central state for Top-level FSM
  """
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['move','use_arm','use_arm_sub','idling','release_obj','done'],
                         input_keys=['idleStartCommandIn','statusMO'],
                         output_keys=['idleNavGoalOut', 'idleArmGoalOut'] )

    # Initialize variables for transition to other states
    self.counter = 0
    self.transitionList = ['move','use_arm_sub','move','use_arm','release_obj','use_arm','done']
    self.transitionCount = -1                   # Index number to call the next transition from transitionList
    self.receivedStartCommand = False           # This will be set to true after experiment speech command received

    # Initialize Speech Synthesizer from HLPR Speech Stack
    self.ss = speech_synthesizer.SpeechSynthesizer()

    # Initialize Manipulator class from HLPR Manipulation Stack
    self.manip = Manipulator()

    # Initialize Speech Listener Client from HLPR Speech Stack
    self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM)
    if self.service_topic is None:
      rospy.logerr("Exiting: No speech topic given, is speech listener running?")
      exit()

    rospy.logwarn("Waiting for speech service")
    rospy.wait_for_service(self.service_topic)
    self.speech_service = rospy.ServiceProxy(self.service_topic, SpeechService)
    rospy.logwarn("Speech service loaded")

    self.ss.say("I am awake, I am awake")


  def execute(self, userdata):
    """
    Define the handshake speech and state transitions from Idle
    """

    # Pull the speech commands received
    try:
      response = self.speech_service(True)
      speech_com = response.speech_cmd
    except rospy.ServiceException:
      speech_com = None


    # Print speech commands recieved and robot replies
    if speech_com is not None:
      print speech_com

    if speech_com == 'START_EXP':
      self.receivedStartCommand = True
      self.ss.say("Here we go")
    elif speech_com == 'GREETING':
      self.ss.say("Hello Human")
    elif speech_com == 'SMALL_TALK':
      self.ss.say("Excited as always")
    elif speech_com == 'HEAR_CHECK':
      self.ss.say("Yes, can you hear me?")
    elif speech_com == 'OPEN_HAND':
      self.manip.gripper.open()
      self.ss.say("Okay")
    elif speech_com == 'CLOSE_HAND':
      self.manip.gripper.close()
      self.ss.say("Okay")

    # Execute transition only if asked through speech or input_key start command
    # Define state transitions
    if not (userdata.idleStartCommandIn or self.receivedStartCommand):
      time.sleep(0.5)
      return 'idling'
    else:
      rospy.loginfo('Executing state Idle')
      self.transitionCount += 1
      if self.transitionCount == 0:           # transition 1: Idle -> navigate to 'table'
        userdata.idleNavGoalOut = 'table'
      elif self.transitionCount == 2:         # transition 3: Idle -> navigate to 'person'
        userdata.idleNavGoalOut = 'person'
      else:
        userdata.idleNavGoalOut = None
      if self.transitionCount == 1:           # transition 2: Idle -> plan and execute object_manip for 'object'
        userdata.idleArmGoalOut = 'object'
      elif self.transitionCount == 3:
        if userdata.statusMO == 'failed':     # transition 4: Skip to transition 5 if object_manip failed
          userdata.idleArmGoalOut = None
          self.transitionCount += 1
        else:                                 # transition 4: Idle -> execute arm traj to person if object_manip success
          userdata.idleArmGoalOut = 'person'
      elif self.transitionCount == 4:         # transition 5: Idle -> Open/Close gripper
        return  self.transitionList[self.transitionCount]
      elif self.transitionCount == 5:         # transition 6: Idle -> excecute arm retract trajectory
        userdata.idleArmGoalOut = 'retract'
      if(self.transitionList[self.transitionCount] == 'done'):
        self.ss.say("I am done. Have a nice day")
        time.sleep(1);
      return self.transitionList[self.transitionCount]




def main():
  rospy.init_node('smach_example_state_machine')

  # Create a Top-level state machine
  sm = smach.StateMachine(outcomes=['end'])

  # Set default values of userdata in Top-level State Machine
  sm.userdata.startCommand = False
  sm.userdata.object = 'coffee bag'
  sm.userdata.navGoal = None
  sm.userdata.armGoal = None
  sm.userdata.tra = 'handOff'
  sm.userdata.gripperCommand = 'Open'
  sm.userdata.handOffSpeech = True

  withR = True                                      # Flag set true if the mobile base is used

  # Open the container and Add States in Top-Level State Machine
  with sm:
    smach.StateMachine.add('Idle', IdleState(),
                            transitions={'idling':'Idle',
                                         'move':'GoToPoint',
                                         'use_arm':'ExecTra',
                                         'use_arm_sub':'manip_sub',
                                         'release_obj':'ReleaseObject',
                                         'done':'end'},
                            remapping={'idleStartCommandIn':'startCommand',
                                       'idleNavGoalOut':'navGoal',
                                       'idleArmGoalOut':'armGoal',
                                       'idleArmGoalOut':'tra'})

    smach.StateMachine.add('GoToPoint', GoToPointState(withRobot=True),
                            transitions={'succeeded':'Idle',
                                         'aborted':'end'},
                            remapping={'navGoalIn':'navGoal'})

    smach.StateMachine.add('ExecTra', ExecuteTrajectoryState(),
                            transitions={'succeeded':'Idle',
                                         'failed':'end',
                                         'aborted':'end'},
                                         remapping={'trajectoryIn':'tra'})

    smach.StateMachine.add('ReleaseObject', UseGripperState(),
                            transitions={'succeeded':'Idle',
                                         'failed':'end',
                                         'aborted':'end'},
                            remapping={'gripperCommandIn':'gripperCommand',
                                       'waitForSpeech':'handOffSpeech',
                                       'resultOut':'graspResult'})


    # Create Manipulation Sub State Machine
    manip_sub = smach.StateMachine(outcomes=['succeededMO','failedMO','abortedMO'],
                                   input_keys  = ['armGoalMO','objectMO'],
                                   output_keys = ['statusMO'])

    # Set default values of userdata in Manipulation Sub State Machine
    manip_sub.userdata.armGoalMO = None
    manip_sub.userdata.objectMO = sm.userdata.object    # Object name is the one specified in top-level SM
    manip_sub.userdata.tra = None
    manip_sub.userdata.objectLocation = None
    manip_sub.userdata.gripperCommand = None
    manip_sub.userdata.statusMO = None
    manip_sub.userdata.pickSuccess = None
    manip_sub.userdata.pathFound = False
    manip_sub.userdata.targetPose = None
    manip_sub.userdata.graspResult = False

# Get the arm goal into the sub state

# mixing and matching sequential transitions and 1 main state type structures
    with manip_sub:
      smach.StateMachine.add('ManipulateObjectMain', ManipulateObjectMainState(),
                              transitions={'findObject':'FindObject',
                                           'planPath':'PlanTraj',
                                           'moveArm':'ExecTra',
                                           'gripper':'UseGripper',
                                           'pickRetract':'PickAndRetract',
                                           'failed':'failedMO',
                                           'succeeded':'succeededMO',
                                           'aborted':'abortedMO'},
                              remapping={'armGoalIn':'armGoalMO',
                                         'objectLocationIn':'objectLocation',
                                         'gripperCommandOut':'gripperCommand',
                                         'traOut':'tra',
                                         'pathFoundIn':'pathFound',
                                         'graspResultIn':'graspResult',
                                         'pickResultIn':'pickResult',
                                         'targetPoseIn':'targetPose',
                                         'statusOut':'statusMO'})

      #smach.StateMachine.add('FindObject', FindObjectState(withRobot=withR, tr_root = 'odom'),
      smach.StateMachine.add('FindObject', FindObjectState(withRobot=withR, tr_root = 'base_link'),
                              transitions={'succeeded':'ManipulateObjectMain',
                                           #'succeeded':'PlanTraj',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'objectIn':'objectMO',
                                         'objectLocationOut':'objectLocation'})

      smach.StateMachine.add('PlanTraj', PlanTrajectoryState(),
                              transitions={'succeeded':'ManipulateObjectMain',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'objectLocationIn':'objectLocation',
                                         'traOut':'tra',
                                         'pathFoundOut':'pathFound',
                                         'targetPoseOut':'targetPose'})

      smach.StateMachine.add('ExecTra', ExecuteTrajectoryState(),
                              transitions={'succeeded':'ManipulateObjectMain',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'trajectoryIn':'tra'})

      #smach.StateMachine.add('PickAndRetract', CompositePickAndRetractState(ik_root = 'odom'),
      smach.StateMachine.add('PickAndRetract', CompositePickAndRetractState(ik_root = 'base_link'),
                              transitions={'succeeded':'ManipulateObjectMain',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'initArmPoseIn':'targetPose', #should this be taken from forward kinematics?
                                         'pickResultOut':'pickResult'})

      smach.StateMachine.add('UseGripper', UseGripperState(),
                              transitions={'succeeded':'ManipulateObjectMain',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'gripperCommandIn':'gripperCommand',
                                         'resultOut':'graspResult'})


    # Add the sub state machine to the top-level state machine
    smach.StateMachine.add('manip_sub', manip_sub,
                            transitions={'succeededMO':'Idle',
                                         'failedMO':'Idle',
                                         'abortedMO':'end'},
                            remapping={'armGoalMO':'armGoal',
                                       'objectMO':'object'})


  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  # Execute SMACH plan
  outcome = sm.execute()
  print outcome
  sis.stop()



if __name__ == '__main__':
  main()
