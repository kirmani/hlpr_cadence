#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc


class ExtendTransition(Transition):
	actionType = ""
	def fire(self,binding):
		if(self.actionType == "start"):
			if(self.enabled(binding)):
				p.start()
				Transition.fire(self,binding)
		elif(self.actionType == "interrupt"):
			if(self.enabled(binding)):
				p.terminate()
				Transition.fire(self,binding)
		elif(self.actionType == "finish"):
			running = p.is_alive()
			if(self.enabled(binding) and not running):
				print("finishing")
				Transition.fire(self,binding)

	def enabled(self,binding):
		rospy.wait_for_service('do_petri_net_arc')
		try:
			do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc',DoPetriNetArc)
			return do_petri_net_arc("guard","owned",None,"floor").guard and Transitioself.petri_net_.enabled(binding)
		except rospy.ServiceException, e:
			print("Service call failed: %s"%e)
			return False

def sayThings(text):
	rate = 99/2
	pitch = 99/2
	rate = 80+(370-80)*int(rate)/100
	subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

class ActionProcess:
  def __init__(self, name):
    self.name_ = name
    self.petri_net_ = PetriNet(self.name_)

    self.places_ = []
    self.places_.append(Place('queue',[]))
    self.places_.append(Place('started',[]))
    self.places_.append(Place('interrupted',[]))
    self.places_.append(Place('finished',[]))

    self.transitions_ = []
    self.transitions_.append(ExtendTransition('start'))
    self.transitions_.append(ExtendTransition('interrupt'))
    self.transitions_.append(ExtendTransition('finish'))

    for place in self.places_:
      self.petri_net_.add_place(place)
    for transition in self.transitions_:
      self.petri_net_.add_transition(transition)

    self.petri_net_.add_input("queue", "start", Variable('act'))
    self.petri_net_.add_output("started", "start", Expression("act"))
    self.petri_net_.add_input("started", "interrupt", Variable("act"))
    self.petri_net_.add_input("started", "finish", Variable("act"))
    self.petri_net_.add_input("interrupted", "finish", Variable("act"))
    self.petri_net_.add_output("finished", "finish", Expression("act"))
    self.petri_net_.add_output("interrupted", "interrupt", Expression("act"))

  def AddAction(self, action):
    # Queue up action token.
    self.petri_net_.place('queue').add(action.name)

    # Place resource tokens in requested place.
    for token in action.preconditions:
      FirePetriNetArc('request_robot', token)

  def Run(self):
    while True:
      for transition in self.transitions_:
        if len(transition.modes()) > 0:
          transition.fire(transition.modes().pop())
          # print("Action marking: %s" % str(self.petri_net_.get_marking()))

def FirePetriNetArc(transition, token):
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
    do_petri_net_arc('fire', None, transition, token)
  except rospy.ServiceException, e:
    print("Service call failed: %s" % e)

def CheckGuard(place, token):
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
    return do_petri_net_arc('guard', place, None, token)
  except rospy.ServiceException, e:
    print("Service call failed: %s" % e)
    return False


class Action:
  def __init__(self, name, entities, preconditions, postconditions):
    self.name = name
    self.entities = entities
    self.preconditions = preconditions
    self.postconditions = postconditions

def main():
	global p 
	p = Process(target=sayThings,args=("Hello world, how are you doing, this is a long sentence"))
	action_process = ActionProcess('speech_action_process')
	action = Action('speech', ['floor'], {'floor': 'true'}, {'floor': 'true'})
	action_process.AddAction(action)
	action_process.Run()

if __name__ == '__main__':
  main()

