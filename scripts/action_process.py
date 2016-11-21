#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc
from petri_net import *
from resource_controller import ResourceControllerApi

class StartTransition(PetriNetTransition):
  def __init__(self, name, action, queue, started):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.queue_ = queue
    self.started_ = started

  def fire(self):
    print("Starting action: %s" % self.action_.name)
    p.start()
    self.queue_.RemoveToken(self.action_.name)
    self.started_.AddToken(self.action_.name)

  def activated(self):
    if not self.queue_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ResourceControllerApi.CheckGuard('owned_robot', resource):
        return False
    return True

class InterruptTransition(PetriNetTransition):
  def __init__(self, name, action, started, interrupted):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.started_ = started
    self.interrupted_ = interrupted

  def fire(self):
    print("Interrupting action: %s" % self.action_.name)
    p.terminate()
    self.started_.RemoveToken(self.action_.name)
    self.interrupted_.AddToken(self.action_.name)

  def activated(self):
    if not self.started_.HasToken(self.action_.name):
      return False
    for resource in self.action_.preconditions:
      if not ResourceControllerApi.CheckGuard('owned_robot', resource):
        return True
    return False

class FinishTransition(PetriNetTransition):
  def __init__(self, name, action, started, interrupted, finished):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.started_ = started
    self.interrupted_ = interrupted
    self.finished_ = finished

  def fire(self):
    print("Finishing action: %s" % self.action_.name)
    if self.started_.HasToken(self.action_.name):
      self.started_.RemoveToken(self.action_.name)
      ResourceControllerApi.AddResourceToPlace('requested_robot', 'floor')
    if self.interrupted_.HasToken(self.action_.name):
      self.interrupted_.RemoveToken(self.action_.name)
    self.finished_.AddToken(self.action_.name)

  def activated(self):
    return (self.started_.HasToken(self.action_.name) \
        or self.interrupted_.HasToken(self.action_.name)) \
        and not p.is_alive()

class SeizeRobotTransition(PetriNetTransition):
  def __init__(self, name, action):
    PetriNetTransition.__init__(self, name)
    self.action_ = action

  def fire(self):
    # Remove resources from requested, and put resource tokens in requested
    # place.
    ResourceControllerApi.RemoveResourceFromPlace('requested_robot', 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('free', 'floor')
    ResourceControllerApi.AddResourceToPlace('owned_robot', 'floor')

  def activated(self):
    if not PetriNetTransition.activated(self):
      return False
    for resource in self.action_.preconditions:
      if not (ResourceControllerApi.CheckGuard('requested_robot', resource)
              and ResourceControllerApi.CheckGuard('free', resource)):
        return False
    return True

class RequestRobotTransition(PetriNetTransition):
  def __init__(self, action):
    PetriNetTransition.__init__(self, 'request_robot')
    self.action_ = action
    self.already_requested_ = False

  def fire(self):
    # Place resource tokens in requested place.
    print("Requesting resources for action: %s" % self.action_.name)
    ResourceControllerApi.AddResourceToPlace('requested_robot', 'floor')

  def activated(self):
    if not self.already_requested_:
      self.already_requested_ = True
      return True
    return False

def sayThings(text):
  while True:
    rate = 99/2
    pitch = 99/2
    rate = 80+(370-80)*int(rate)/100
    subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

class ActionProcess(PetriNet):
  def __init__(self, name, action):
    PetriNet.__init__(self, name)
    self.action_ = action

    # Places.
    queue = PetriNetPlace('queue')
    started = PetriNetPlace('started')
    interrupted = PetriNetPlace('interrupted')
    self.finished_ = PetriNetPlace('finished')

    # Transitions.
    self.transitions_.append(RequestRobotTransition(action))
    self.transitions_.append(StartTransition('start', action, queue, started))
    self.transitions_.append(SeizeRobotTransition('seize_robot', action))
    self.transitions_.append(
        InterruptTransition('interrupt', action, started, interrupted))
    self.transitions_.append(
        FinishTransition('finish', action, started,interrupted, self.finished_))

    # Put action token in queue.
    queue.AddToken(self.action_.name)

  def EndCondition(self):
    return rospy.is_shutdown() or self.finished_.HasToken(self.action_.name)

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

