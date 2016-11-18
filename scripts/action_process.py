#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String


class ExtendTransition(Transition):
	actionType = 0
	def fire(self,binding):
		Transition.fire(self,binding)
		if(self.actionType == "start"):
			#start speech
			getAvailable()
		elif(self.actionType == "hold"):
			print("hold")
		elif(self.actionType == "resume"):
			print("resume")
		elif(self.actionType == "pause"):
			print("pause")
		elif(self.actionType == "interrupt"):
			print("interrupt")
		elif(self.actionType == "finish"):
			print("finish")

def tryStart(data):
	print("trying")

def getAvailable():
	print("start")
	rospy.init_node('startListen',anonymous=True)
	rospy.Subscriber("talk",String,tryStart)

def createNet():
	n = PetriNet("Speech")
	n.add_place(Place('queue',["floor"]))
	n.add_place(Place('started',["act"]))
	n.add_place(Place('shouldPause',[]))
	n.add_place(Place('shouldInterrupt',[]))
	n.add_place(Place('paused',[]))
	n.add_place(Place('interrupted',[]))
	n.add_place(Place('finished',[]))
	n.add_transition(ExtendTransition("start"))
	n.transition('start').actionType = "start"
	n.add_transition(ExtendTransition("hold"))
	n.transition('hold').actionType = "hold"
	n.add_transition(ExtendTransition("resume"))
	n.transition('resume').actionType = "resume"
	n.add_transition(ExtendTransition("pause"))
	n.transition('pause').actionType = "pause"
	n.add_transition(ExtendTransition("interrupt"))
	n.transition('interrupt').actionType = "interrupt"
	n.add_transition(ExtendTransition("finish"))
	n.transition('finish').actionType = "finish"
	n.add_input("queue","start",Variable("floor"))
	n.add_output("started","start",Expression("floor"))
	n.add_input("started","hold",Variable("act"))
	n.add_output("shouldPause","hold",Expression("act"))
	n.add_output("shouldInterrupt","hold",Expression("act"))
	n.add_input("started","finish",Variable("act"))
	n.add_input("interrupted","finish",Variable("act"))
	n.add_output("finished","finish",Expression("act"))
	n.add_input("shouldInterrupt","interrupt",Variable("act"))
	n.add_output("interrupted","interrupt",Expression("act"))
	n.add_input("paused","resume",Variable("act"))
	n.add_output("started","resume",Expression("act"))
	n.add_output("shouldInterrupt","resume",Expression("act"))
	n.add_input("shouldPause","pause",Variable("act"))
	n.add_output("paused","pause",Expression("act"))

	return n



def main():
	n = createNet()
	print(n.transition('start').modes())
	n.transition('start').fire(n.transition("start").modes().pop())

main()

#print(n.place('p').tokens)
#print(n.get_marking())

