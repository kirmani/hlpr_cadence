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
      return do_petri_net_arc("guard","owned","floor").guard and Transitioself.petri_net_.enabled(binding)
    except rospy.ServiceException, e:
      print("Service call failed: %s"%e)
      return False

def sayThings(text, rate=RATE_DEFAULT, pitch=PITCH_DEFAULT):
  rate = 80+(370-80)*int(rate)/100
  subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)


class ActionProcess:
  def __init__(self, name):
    self.name_ = name
    self.petri_net_ = PetriNet(self.name_)

    places = []
    places.append(Place('queue',["act"]))
    places.append(Place('started',[]))
    places.append(Place('interrupted',[]))
    places.append(Place('finished',[]))

    transitions = []
    transitions.append(ExtendTransition("start"))
    transitions.append(ExtendTransition("interrupt"))
    transitions.append(ExtendTransition("finish"))

    for place in places:
      self.petri_net_.add_place(place)
    for transition in transitions:
      self.petri_net_.add_transition(transition)

    self.petri_net_.add_input("queue", "start", Variable("act"))
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
  action_process = ActionProcess('speech_action_process')

  action = Action('speech', ['floor'], {'floor': 'true'}, {'floor': 'true'})
  action_process.AddAction(action)

if __name__ == '__main__':
  main()

