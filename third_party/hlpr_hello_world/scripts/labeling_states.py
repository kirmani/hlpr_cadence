# define state GetLabelLearning
import roslib
import rospy
import smach
import smach_ros

from hlpr_speech_recognition.speech_listener import SpeechListener
from hlpr_speech_msgs.srv import SpeechService
#from hlpr_speech_synthesis import speech_synthesizer
from hlpr_manipulation_utils.manipulator import *

import sys
import time
import os
from Tkinter import *
from PIL import Image, ImageTk

class GetLabelTesting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        #self.ss = speech_synthesizer.SpeechSynthesizer()
        self.counter = 0
        

    def execute(self, userdata):
        rospy.loginfo('Executing state GetLabelLearning')
        if self.counter < 3:
            self.counter += 1
            return 'succeeded'
        else:
            return 'failed'


# define state GetLabelTesting
class GetLabelLearning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.ss = speech_synthesizer.SpeechSynthesizer()
        
        

    def execute(self, userdata):
        rospy.loginfo('Executing state GetLabelTesting')
        #self.ss.say("What is this object?")
        #self.ss.say("What can I do with it?")
        #time.sleep(1.5)
        #self.ss.say('Thank you for the information!')
        return 'succeeded'
