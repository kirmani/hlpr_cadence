#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc
from petri_net import *
from resource_controller import ResourceControllerApi
import signal
import sys

class StartTransition(PetriNetTransition):
  def __init__(self, name, action, queue, started):
    PetriNetTransition.__init__(self, name)
    self.action_ = action
    self.queue_ = queue
    self.started_ = started

  def fire(self):
    print("Starting action: %s" % self.action_.name)
    self.action_.Start()
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
    self.action_.Interrupt()
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
        and self.action_.IsFinished()

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

  def Start(self):
    pass

  def Interrupt(self):
    pass

  def IsFinished(self):
    return True

class Speak(Action):
  def __init__(self, rate, pitch, text):
    Action.__init__(self, 'speech', ['floor'], {'floor': True}, {'floor': False})
    self.rate_ = rate
    self.pitch_ = pitch
    self.text_ = text
    self.process_ = Process(target= self.Speak_)
    self.subprocess_ = None

  def Start(self):
    self.process_.start()

  def Interrupt(self):
    os.killpg(os.getpgid(self.process_.pid), signal.SIGTERM)

  def IsFinished(self):
    return not self.process_.is_alive()

  def sigterm_handler(_signo, _stack_frame):
        # Raises SystemExit(0):
            sys.exit(0)

  def Speak_(self):
      cmd = ['espeak', '-p', str(self.pitch_), '-s', str(self.rate_), '-v',
             'en', self.text_]
      proc = subprocess.Popen(cmd).wait()

def main():
	if(len(sys.argv) > 1):
		sentence = sys.argv[1]
	else:
		sentence = "Hello world"
	rate = 150
	pitch = 50
	speak = Speak(rate, pitch, sentence)
	action_process = ActionProcess('speech_action_process', speak)
	action_process.Run()

if __name__ == '__main__':
  main()

