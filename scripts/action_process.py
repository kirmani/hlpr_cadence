#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc
from petri_net import *

class StartTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    if(self.activated()):
      p.start()
      PetriNetTransition.fire(self)

  def activated(self):
    if not PetriNetTransition.activated(self):
      return False
    for resource in self.action_.preconditions:
      if not (CheckGuard('owned_robot', resource) and CheckGuard('free', resource)):
        return False
    return True

class InterruptTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    if(self.activated()):
      p.terminate()
      PetriNetTransition.fire(self)

  def activated(self):
    if not PetriNetTransition.activated(self):
      return False
    for resource in self.action_.preconditions:
      if not CheckGuard('owned_robot', resource):
        return True
    return False

class FinishTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    pass
    # for token in self.action_.postconditions:
    #   AddResourceToPlace('requested_robot', token)

  def activated(self):
    return not p.is_alive()

class SeizeRobotTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    if(self.activated()):
      # Remove resources from requested, and put resource tokens in requested
      # place.
      RemoveResourceFromPlace('requested_robot', 'floor')
      AddResourceToPlace('owned_robot', 'floor')
      PetriNetTransition.fire(self)

  def activated(self):
    if not PetriNetTransition.activated(self):
      return False
    for resource in self.action_.preconditions:
      if not (CheckGuard('owned_robot', resource) and CheckGuard('free', resource)):
        return False
    return True

def sayThings(text):
  rate = 99/2
  pitch = 99/2
  rate = 80+(370-80)*int(rate)/100
  subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

class ActionProcess:
  def __init__(self, name, action):
    self.name_ = name
    self.places_ = []
    self.transitions_ = []
    self.action_ = action
    self.start_place_ = None
    self.end_place_ = None

    # Places.
    queue_place = PetriNetPlace('queue')
    started_place = PetriNetPlace('started')
    interrupted_place = PetriNetPlace('interrupted')
    finished_place = PetriNetPlace('finished')

    # Transitions.
    start_transition = StartTransition('start', action)
    interrupt_transition = InterruptTransition('interrupt', action)
    seize_robot_transition = SeizeRobotTransition('seize_robot', action)
    finish_transition = FinishTransition('finish', action)

    # Guard arcs.
    queue_place.AddOutput(seize_robot_transition)
    queue_place.AddOutput(start_transition)
    started_place.AddOutput(interrupt_transition)
    started_place.AddOutput(finish_transition)
    interrupted_place.AddOutput(finish_transition)

    # Firing arcs.
    start_transition.AddOutput(started_place)
    interrupt_transition.AddOutput(interrupted_place)
    finish_transition.AddOutput(finished_place)

    # Set starting and ending place.
    self.start_place_ = queue_place
    self.end_place_ = finished_place

  def Run(self):
    # Put action token in queue.
    action_token = PetriNetToken(self.action_.name, self.start_place_)

    # Place resource tokens in requested place.
    for token in self.action_.preconditions:
      AddResourceToPlace('requested_robot', token)

    while not rospy.is_shutdown() and not action_token.GetLocation() == self.end_place_:
      for transition in action_token.GetLocation().GetTransitions():
        if transition.activated():
          transition.fire()
          places = transition.GetOutputs()
          if len(places) == 1:
            action_token.SetLocation(places[0])
            print("current location: %s" % action_token.GetLocation().name)

def RemoveResourceFromPlace(place, token):
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
    return do_petri_net_arc('fire', place, 'remove', token).guard
  except rospy.ServiceException, e:
    print("Service call failed: %s" % e)
    return False

def AddResourceToPlace(place, token):
  rospy.wait_for_service('do_petri_net_arc')
  try:
    do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
    return do_petri_net_arc('fire', place, 'add', token).guard
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

