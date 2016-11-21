#!/usr/bin/env python

from snakes.nets import *
import rospy
from std_msgs.msg import String
import os
import subprocess
from multiprocessing import Process
from hlpr_cadence.srv import DoPetriNetArc

class ActionProcessNode(object):
  def __init__(self, name):
    self.name = name
    self.outputs_ = []

  def AddOutput(self, node):
    self.outputs_.append(node)

  def GetOutputs(self):
    return self.outputs_

class ActionProcessTransition(ActionProcessNode):
  def fire(self):
    if self.activated():
      pass

  def activated(self):
    return True

class RemoteTransition(ActionProcessTransition):
  def fire(self):
    firing_succeeded = FirePetriNetArc(self.name, 'floor')
    print ("Firing succeeded: %s" % firing_succeeded)
    return firing_succeeded

class StartTransition(ActionProcessTransition):
  def __init__(self, name):
    ActionProcessTransition.__init__(self, name)
    self.resource_inputs_ = []

  def fire(self):
    if(self.activated()):
      p.start()
      ActionProcessTransition.fire(self)

  def activated(self):
    if not ActionProcessTransition.activated(self):
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

class InterruptTransition(ActionProcessTransition):
  def __init__(self, name):
    ActionProcessTransition.__init__(self, name)
    self.resource_inputs_ = []

  def fire(self):
    if(self.activated()):
      p.terminate()
      ActionProcessTransition.fire(self)

  def activated(self):
    if not ActionProcessTransition.activated(self):
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

class FinishTransition(ActionProcessTransition):
  def activated(self):
    return not p.is_alive()

def sayThings(text):
  rate = 99/2
  pitch = 99/2
  rate = 80+(370-80)*int(rate)/100
  subprocess.call(["espeak","-p",str(pitch),"-s",str(rate),"-v","en",text],stdout=subprocess.PIPE)

class ActionProcessPlace(ActionProcessNode):
  def GetTransitions(self):
    return self.outputs_

class ActionToken(ActionProcessNode):
  def __init__(self, name, location):
    self.location_ = location

  def GetLocation(self):
    return self.location_

  def SetLocation(self, location):
    self.location_ = location

class ActionProcess:
  def __init__(self, name, action):
    self.name_ = name
    self.petri_net_ = PetriNet(self.name_)
    self.places_ = []
    self.transitions_ = []
    self.action_ = action
    self.start_place_ = None
    self.end_place_ = None

    # Places.
    queue_place = ActionProcessPlace('queue')
    started_place = ActionProcessPlace('started')
    interrupted_place = ActionProcessPlace('interrupted')
    finished_place = ActionProcessPlace('finished')
    # self.places_.append(Place('queue',[]))
    # self.places_.append(Place('started',[]))
    # self.places_.append(Place('interrupted',[]))
    # self.places_.append(Place('finished',[]))

    # Transitions.
    start_transition = StartTransition('start')
    start_transition.AddResourceInput('owned_robot')
    interrupt_transition = InterruptTransition('interrupt')
    interrupt_transition.AddResourceInput('owned_robot')
    finish_transition = FinishTransition('finish')
    seize_robot_transition = RemoteTransition('seize_robot')
    # self.transitions_.append(start_transition)
    # self.transitions_.append(interrupt_transition)
    # self.transitions_.append(finish_transition)
    # self.transitions_.append(RemoteTransition('seize_robot'))

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

    # for place in self.places_:
    #   self.petri_net_.add_place(place)
    # for transition in self.transitions_:
    #   self.petri_net_.add_transition(transition)

    # Arcs.
    # self.petri_net_.add_input("queue", "start", Variable('act'))
    # self.petri_net_.add_output("started", "start", Expression("act"))
    # self.petri_net_.add_input("started", "interrupt", Variable("act"))
    # self.petri_net_.add_input("started", "finish", Variable("act"))
    # self.petri_net_.add_input("interrupted", "finish", Variable("act"))
    # self.petri_net_.add_output("finished", "finish", Expression("act"))
    # self.petri_net_.add_output("interrupted", "interrupt", Expression("act"))
    # self.petri_net_.add_input('queue', 'seize_robot', Variable('act'))

  def Run(self):
    # Put action token in queue.
    action_token = ActionToken(self.action_.name, self.start_place_)
    print("Added action token to queue %s" % str(self.petri_net_.get_marking()))

    # Place resource tokens in requested place.
    for token in self.action_.preconditions:
      FirePetriNetArc('request_robot', token)

    while not rospy.is_shutdown() and not action_token.GetLocation() == self.end_place_:
      for transition in action_token.GetLocation().GetTransitions():
        if transition.activated():
          transition.fire()
          places = transition.GetOutputs()
          if len(places) == 1:
            action_token.SetLocation(places[0])

    # while not rospy.is_shutdown():
    #   for transition in self.transitions_:
    #     if len(transition.modes()) > 0:
    #       binding = transition.modes().pop()
    #       if (transition.activated(binding)):
    #         print("Markings before firing transition (%s): %s"
    #             % (transition.name, self.petri_net_.get_marking()))
    #         transition.fire(binding)
    #         print("Markings after firing transition (%s): %s"
    #             % (transition.name, self.petri_net_.get_marking()))


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

