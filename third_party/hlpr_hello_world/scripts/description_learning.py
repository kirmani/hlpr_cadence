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
from labeling_states import *

#comments:
#execute might take all the cycles, not very c6 like, see if smach has an updateOnce/executeOnce option

class IdleState(smach.State):
	"""
	Central state for Top-level FSM
	"""
	def __init__(self):
		smach.State.__init__(self,
                         outcomes=['idling','find_obj','get_obj','done'],
                         input_keys=['idleStartCommandIn','statusMO'],
                         output_keys=['idleNavGoalOut', 'idleArmGoalOut'] )

	# Initialize variables for transition to other states
		self.counter = 0
		self.transitionList = ['idling','find_obj','get_obj','done']
		self.transitionCount = -1                   # Index number to call the next transition from transitionList
		self.receivedStartCommand = False           # This will be set to true after experiment speech command received

    # Initialize Speech Synthesizer from HLPR Speech Stack
		self.ss = speech_synthesizer.SpeechSynthesizer()

    # Initialize Speech Listener Client from HLPR Speech Stack
		self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM)
		if self.service_topic is None:
			rospy.logerr("Exiting: No speech topic given, is speech listener running?")
			exit()

		rospy.logwarn("Waiting for speech service")
		rospy.wait_for_service(self.service_topic)
		self.speech_service = rospy.ServiceProxy(self.service_topic, SpeechService)
		rospy.logwarn("Speech service loaded")
		self.ss.say("Wow! I am alive.")


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

		speech_com = 'START_EXP'

		if speech_com == 'START_EXP':
		  self.receivedStartCommand = True
		  self.transitionCount = 1
		  self.ss.say("Here we go")
		elif speech_com == 'GREETING':
		  self.ss.say("Hello, Human")
		elif speech_com == 'SMALL_TALK':
		  self.ss.say("Excited as always")
		elif speech_com == 'HEAR_CHECK':
		  self.ss.say("Yes, can you hear me?")
		elif speech_com == 'OPEN_HAND':
		  self.ss.say("Nope")
		elif speech_com == 'CLOSE_HAND':
		  self.transitionCount = 3
		  self.ss.say("Nope")

		# Execute transition only if asked through speech or input_key start command
		# Define state transitions
		if not (userdata.idleStartCommandIn or self.receivedStartCommand):
		  time.sleep(0.5)
		  return 'idling'
		else:
		  self.receivedStartCommand = False
		  rospy.loginfo('Executing state Idle')
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
  sm.userdata.object = 'mug'
  sm.userdata.navGoal = None
  sm.userdata.armGoal = None
  sm.userdata.tra = 'handOff'
  sm.userdata.gripperCommand = 'Open'
  sm.userdata.handOffSpeech = True

  withR = True                                     # Flag set true if the mobile base is used

  # Open the container and Add States in Top-Level State Machine
  with sm:
    smach.StateMachine.add('Idle', IdleState(),
                            transitions={'idling':'Idle',
                                         'find_obj':'FindObjectLearning',
                                         'get_obj': 'GetLabelTesting',
                                         'done':'end'},
                            remapping={'idleStartCommandIn':'startCommand',
                                       'idleNavGoalOut':'navGoal',
                                       'idleArmGoalOut':'armGoal',
                                       'idleArmGoalOut':'tra'})

    # learning loop

    smach.StateMachine.add('FindObjectLearning', FindObjectState(withRobot=withR, tr_root = 'base_link'),
                              transitions={'succeeded':'PlanTrajLearning',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'objectIn':'object',
                                         'objectLocationOut':'objectLocation'})

    smach.StateMachine.add('PlanTrajLearning', PlanTrajectoryState(),
                              transitions={'succeeded':'ExecTraLearning',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'objectLocationIn':'objectLocation',
                                         'traOut':'tra',
                                         'pathFoundOut':'pathFound',
                                         'targetPoseOut':'targetPose'})

    smach.StateMachine.add('ExecTraLearning', ExecuteTrajectoryState(),
                            transitions={'succeeded':'GetLabelLearning',
                                         'failed':'end',
                                         'aborted':'end'},
                                         remapping={'trajectoryIn':'tra'})

    smach.StateMachine.add('GetLabelLearning', GetLabelLearning(),
                              transitions={'succeeded':'Retract', #PlanTraj2Learning',
                                           'aborted':'end'})

    smach.StateMachine.add('Retract', CompositePickAndRetractState(ik_root = 'base_link'),
                              transitions={'succeeded':'Idle',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'initArmPoseIn':'targetPose', #should this be taken from forward kinematics?
                                         'pickResultOut':'pickResult'})

    '''smach.StateMachine.add('PlanTraj2Learning', PlanTrajectoryState(),
                              transitions={'succeeded':'ExecTra2Learning',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'objectLocationIn':'objectLocation',
                                         'traOut':'tra',
                                         'pathFoundOut':'pathFound',
                                         'targetPoseOut':'targetPose'})

    smach.StateMachine.add('ExecTra2Learning', ExecuteTrajectoryState(),
                            transitions={'succeeded':'Idle',
                                         'failed':'end',
                                         'aborted':'end'},
                                         remapping={'trajectoryIn':'tra'})'''
    # testing loop

    """smach.StateMachine.add('GetLabelTesting', GetLabelTesting(),
                              transitions={'succeeded':'FindObjectTesting',
                                           'failed':'Idle',                                             
                                           'aborted':'end'})

    smach.StateMachine.add('FindObjectTesting', FindObjectState(withRobot=withR, tr_root = 'base_link'),
                              transitions={'succeeded':'PlanTrajTesting',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'objectIn':'objectMO',
                                         'objectLocationOut':'objectLocation'})

    smach.StateMachine.add('PlanTrajTesting', PlanTrajectoryState(),
                              transitions={'succeeded':'ExecTraTesting',
                                           'failed':'Idle',
                                           'aborted':'end'},
                              remapping={'objectLocationIn':'objectLocation',
                                         'traOut':'tra',
                                         'pathFoundOut':'pathFound',
                                         'targetPoseOut':'targetPose'})

    smach.StateMachine.add('ExecTraTesting', ExecuteTrajectoryState(),
                            transitions={'succeeded':'Idle',
                                         'failed':'end',
                                         'aborted':'end'},
                                         remapping={'trajectoryIn':'tra'})"""


  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  # Execute SMACH plan
  outcome = sm.execute()
  print outcome
  rospy.spin()
  sis.stop()



if __name__ == '__main__':
  main()
