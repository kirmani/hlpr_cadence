#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


"""
Keyboard listener.
"""
import fcntl
import os
import sys
import termios
import time

from std_srvs.srv import Empty 
from hlpr_speech_msgs.msg import StampedString, SpeechCommand
class GuiListener(ResourceListener):

  def StartListening(self):
    print("Listening for GUI input.")

	def Listen(self):
		rospy.init_node('gui_listen', anonymous=True)
		rospy.Subscriber("hlpr_speech_commands", StampedString, Poll)
		rospy.spin()
		
  def Poll(self):
		if(data.keyphrase == "OBJECT ONE SEIZED"):
			return False
		elif(data.keyphrase == "OBJECT ONE FREE"):
			return True
	
