#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc

class RemoteTransition(Transition):
  def fire(self, binding):
    firing_succeeded = FirePetriNetArc(self.name, 'floor')
    print ("Firing succeeded: %s" % firing_succeeded)
    return firing_succeeded

class StartTransition(Transition):
  def __init__(self, name, guard=None):
    Transition.__init__(self, name, guard)
    self.resource_inputs_ = []

  def fire(self,binding):
    if(self.activated(binding)):
      p.start()
      Transition.fire(self, binding)

  def activated(self, binding):
    if not Transition.activated(self, binding):
      return False
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      for resource_input in self.resource_inputs_:
        if not do_petri_net_arc('guard', resource_input, None, 'floor').guard:
          return False
      return True
    except rospy.ServiceException, e:
      print("Service call failed: %s"%e)
      return False

  def AddResourceInput(self, resource):
    self.resource_inputs_.append(resource)

class InterruptTransition(Transition):
  def __init__(self, name, guard=None):
    Transition.__init__(self, name, guard)
    self.resource_inputs_ = []

  def fire(self,binding):
    if(self.activated(binding)):
      p.terminate()
      Transition.fire(self, binding)

  def activated(self, binding):
    if not Transition.activated(self, binding):
      return False
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      for resource_input in self.resource_inputs_:
        if not do_petri_net_arc('guard', resource_input, None, 'floor').guard:
          return True
      return False
    except rospy.ServiceException, e:
      print("Service call failed: %s"%e)
      return False

  def AddResourceInput(self, resource):
    self.resource_inputs_.append(resource)

class FinishTransition(Transition):
  def activated(self, binding):
    return not p.is_alive()

class ExtendTransition(Transition):
  def __init__(self, name, guard=None):
    Transition.__init__(self, name, guard)
    self.resource_inputs_ = []

  # actionType = ""
  # def fire(self,binding):
  #   if(self.actionType == "start"):
  #     if(self.activated(binding)):
  #       p.start()
  #       Transition.fire(self,binding)
  #   elif(self.actionType == "interrupt"):
  #     if(self.activated(binding)):
  #       p.terminate()
  #       Transition.fire(self,binding)
  #   elif(self.actionType == "finish"):
  #     running = p.is_alive()
  #     if(self.activated(binding) and not running):
  #       print("finishing")
  #       Transition.fire(self,binding)

  def activated(self, binding):
    if not Transition.activated(self, binding):
      return False
    # print("binding: %s" % binding)
    # return Transition.activated(self, binding)
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      for resource_input in self.resource_inputs_:
        if not do_petri_net_arc('guard', resource_input, None, 'floor').guard:
          return self.name == 'interrupt'
      return not self.name == 'interrupt'
    except rospy.ServiceException, e:
      print("Service call failed: %s"%e)
      return False

  def AddResourceInput(self, resource):
    self.resource_inputs_.append(resource)

def sayThings(text):
  rate = 99/2
  pitch = 99/2
  rate = 80+(370-80)*int(rate)/100
  subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

class ActionProcess:
  def __init__(self, name, action):
    self.name_ = name
    self.petri_net_ = PetriNet(self.name_)
    self.places_ = []
    self.transitions_ = []
    self.action_ = action

    # Places.
    self.places_.append(Place('queue',[]))
    self.places_.append(Place('started',[]))
    self.places_.append(Place('interrupted',[]))
    self.places_.append(Place('finished',[]))

    # Transitions.
    start_transition = StartTransition('start')
    start_transition.AddResourceInput('owned_robot')
    self.transitions_.append(start_transition)
    interrupt_transition = InterruptTransition('interrupt')
    self.transitions_.append(interrupt_transition)
    finish_transition = FinishTransition('finish')
    self.transitions_.append(finish_transition)
    self.transitions_.append(RemoteTransition('seize_robot'))

    for place in self.places_:
      self.petri_net_.add_place(place)
    for transition in self.transitions_:
      self.petri_net_.add_transition(transition)

    # Arcs.
    self.petri_net_.add_input("queue", "start", Variable('act'))
    self.petri_net_.add_output("started", "start", Expression("act"))
    self.petri_net_.add_input("started", "interrupt", Variable("act"))
    self.petri_net_.add_input("started", "finish", Variable("act"))
    self.petri_net_.add_input("interrupted", "finish", Variable("act"))
    self.petri_net_.add_output("finished", "finish", Expression("act"))
    self.petri_net_.add_output("interrupted", "interrupt", Expression("act"))
    self.petri_net_.add_input('queue', 'seize_robot', Variable('act'))

  def Run(self):
    # Queue up action token.
    self.petri_net_.place('queue').add(self.action_.name)
    print("Added action to queue %s" % str(self.petri_net_.get_marking()))

    # Place resource tokens in requested place.
    for token in self.action_.preconditions:
      FirePetriNetArc('request_robot', token)

    while not rospy.is_shutdown():
      for transition in self.transitions_:
        if len(transition.modes()) > 0:
          binding = transition.modes().pop()
          if (transition.enabled(binding)):
            print("Markings before firing transition (%s): %s"
                % (transition.name, self.petri_net_.get_marking()))
            transition.fire(binding)
            print("Markings after firing transition (%s): %s"
                % (transition.name, self.petri_net_.get_marking()))

def FirePetriNetArc(transition, token):
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
    return do_petri_net_arc('fire', None, transition, token).guard
  except rospy.ServiceException, e:
    print("Service call failed: %s" % e)
    return False

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
  p = Process(target=sayThings,args=("Hello world, how are you doing, this is a long sentence",))
  action = Action('speech', ['floor'], {'floor': 'true'}, {'floor': 'true'})
  action_process = ActionProcess('speech_action_process', action)
  action_process.Run()

if __name__ == '__main__':
  main()

