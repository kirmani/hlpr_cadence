import smach
import rospy
import smach_ros
import sys
import time
from modules.perception_module import ObjectSearch
#from hlpr_speech_synthesis import speech_synthesizer


class FindObjectState(smach.State):
  """
  This state searches for the object and returns the pose of object wrt to root frame
  """
  
  def __init__(self, withRobot = True, tr_root = 'base_link'):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys = ['objectIn'],
                         output_keys = ['objectLocationOut'])

    # Initialize ObjectSearch and SpeechSynthesizer classes
    self.objSearch = ObjectSearch(withRobot, tr_root)
    #self.ss = speech_synthesizer.SpeechSynthesizer()

  def execute(self, userdata):
    """
    Sweep Pan and Tilt angles. Search for the object.
    """
    
    rospy.loginfo('Executing state FindObject')
    print 'Scan initiated to find the ' + userdata.objectIn
    #self.ss.say("Scan initiated to find an object")# + userdata.objectIn)

    
    # If object found, return average tranform from root link
    # If object not found, return failure
    if(self.objSearch.objectSearch()):
      self.objSearch.directLookAtObject()
      #self.ss.say('Object found')
      time.sleep(0.2)
      #self.ss.say('Human')
      status = 'succeeded'
      #self.objSearch.tf.updateTransform()
      #userdata.objectLocationOut = self.objSearch.tf.tr
      transform = self.objSearch.tf.getAverageObjectTransform(window_size=10,rate=10)
      userdata.objectLocationOut = transform      # Transform from root to object
      print 'Object found at: ' 
      print transform
    else:
      #self.ss.say('no object found')
      userdata.objectLocationOut = None
      status = 'failed'
    #self.objSearch.pantilt.adjust([0,0],10,10)

    return status

