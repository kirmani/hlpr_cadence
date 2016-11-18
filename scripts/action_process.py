#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc

PITCH_MAX = 99
RATE_MAX = 99
PITCH_DEFAULT = PITCH_MAX/2
RATE_DEFAULT = RATE_MAX/2

class ExtendTransition(Transition):
	actionType = 0
	def fire(self,binding):
		if(self.actionType == "start"):
				if(self.enabled(binding)):
					Transition.fire(self,binding)
					p = Process(target=sayThings,args=("Hello world, how are you doing, this is a long sentence",))
					p.start()
		elif(self.actionType == "interrupt"):
				if(self.enabled(binding)):
					os.system("killall -s STOP espeak")
					Transition.fire(self,binding)
		elif(self.actionType == "finish"):
			#guard
			print("finishing")
	
	def enabled(self,binding):
		rospy.wait_for_service('do_petri_net_arc')
		try:
				do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc',DoPetriNetArc)
				return do_petri_net_arc("guard","owned","floor").guard and Transition.enabled(binding)
			except rospy.ServiceException, e:
				print("Service call failed: %s"%e)
				return False


def sayThings(text, rate=RATE_DEFAULT, pitch=PITCH_DEFAULT):
	rate = 80+(370-80)*int(rate)/100
	subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)


def createNet():
	global n 
	n = PetriNet("Speech")
	n.add_place(Place('queue',["act"]))
	n.add_place(Place('started',[]))
	n.add_place(Place('interrupted',[]))
	n.add_place(Place('finished',[]))

	n.add_transition(ExtendTransition("start"))
	n.transition('start').actionType = "start"
	n.add_transition(ExtendTransition("interrupt"))
	n.transition('interrupt').actionType = "interrupt"
	n.add_transition(ExtendTransition("finish"))
	n.transition('finish').actionType = "finish"


	n.add_input("queue","start",Variable("act"))
	n.add_output("started","start",Expression("act"))
	n.add_input("started","interrupt",Variable("act"))
	n.add_input("started","finish",Variable("act"))
	n.add_input("interrupted","finish",Variable("act"))
	n.add_output("finished","finish",Expression("act")))
	n.add_output("interrupted","interrupt",Expression("act"))



	#n.add_place(Place('shouldPause',[]))
	#n.add_place(Place('shouldInterrupt',[]))
	#n.add_place(Place('paused',[]))

	#n.add_transition(ExtendTransition("hold"))
	#n.transition('hold').actionType = "hold"
	#n.add_transition(ExtendTransition("resume"))
	#n.transition('resume').actionType = "resume"
	#n.add_transition(ExtendTransition("pause"))
	#n.transition('pause').actionType = "pause"

	#n.add_output("shouldPause","hold",Expression("act"))
	#n.add_output("shouldInterrupt","hold",Expression("act"))
	#n.add_input("shouldInterrupt","interrupt",Variable("act"))
	#n.add_input("paused","resume",Variable("act"))
	#n.add_output("started","resume",Expression("act"))
	#n.add_output("shouldInterrupt","resume",Expression("act"))
	#n.add_input("shouldPause","pause",Variable("act"))
	#n.add_output("paused","pause",Expression("act"))
	
def main():
	createNet()
	rospy.wait_for_service('do_petri_net_arc')
	try:
		do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc',DoPetriNetArc)
		ans = do_petri_net_arc("fire","requested","floor")
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

	n.transition('start').fire(n.transition("start").modes().pop())

	while("act" not in n.place('finished')):
		n.transition('interrupt').fire(n.transition("interrupt").modes().pop())
		n.transition('finish').fire(n.transition("finish").modes().pop())


main()

