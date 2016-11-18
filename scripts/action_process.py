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
        Transitioself.petri_net_.fire(self,binding)
        p = Process(target=sayThings,args=("Hello world, how are you doing, this is a long sentence",))
        p.start()
    elif(self.actionType == "interrupt"):
      if(self.enabled(binding)):
        os.system("killall -s STOP espeak")
        Transitioself.petri_net_.fire(self,binding)
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
    transitioself.petri_net_.append(ExtendTransition("start"))
    transitioself.petri_net_.append(ExtendTransition("interrupt"))
    transitioself.petri_net_.append(ExtendTransition("finish"))

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

def main():
  action_process = ActionProcess('speech')
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc',DoPetriNetArc)
    ans = do_petri_net_arc("fire","requested","floor")
  except rospy.ServiceException, e:
    print("Service call failed: %s"%e)

  self.petri_net_.transition('start').fire(self.petri_net_.transition("start").modes().pop())

  while("act" not in self.petri_net_.place('finished')):
    self.petri_net_.transition('interrupt').fire(self.petri_net_.transition("interrupt").modes().pop())
    self.petri_net_.transition('finish').fire(self.petri_net_.transition("finish").modes().pop())


main()

