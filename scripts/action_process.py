#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process

PITCH_MAX = 99
RATE_MAX = 99
PITCH_DEFAULT = PITCH_MAX/2
RATE_DEFAULT = RATE_MAX/2

class ExtendTransition(Transition):
	actionType = 0
	def fire(self,binding):
		Transition.fire(self,binding)
		if(self.actionType == "start"):
			#start speech
			p = Process(target=sayThings,args=("Hello world, how are you doing, this is a long sentence",))
			p.start()
		elif(self.actionType == "hold"):
			print("hold")
		elif(self.actionType == "resume"):
			print("resume")
			os.system("killall -s CONT espeak")
		elif(self.actionType == "pause"):
			os.system("killall -s STOP espeak")
		elif(self.actionType == "interrupt"):
			os.system("killall -s STOP espeak")


def sayThings(text, rate=RATE_DEFAULT, pitch=PITCH_DEFAULT):
	rate = 80+(370-80)*int(rate)/100
	subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

def tryStart(data):
	print("trying")
	if(data == "owned"):
		n.transition('start').fire(n.transition("start").modes().pop())

def getAvailable():
	print("start")
	rospy.init_node('startListen',anonymous=True)
	rospy.Subscriber("talk",String,tryStart)

def createNet():
	global n 
	n = PetriNet("Speech")
	n.add_place(Place('queue',[]))
	n.add_place(Place('started',[]))
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
	n.add_input("queue","start",Variable("act"))
	n.add_output("started","start",Expression("act"))
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





def main():
	createNet()
	getAvailable()
	print("change again")
	

main()

#print(n.place('p').tokens)
#print(n.get_marking())
#print(n.transition('start').modes())

