#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Timed Petri net for resource controller.
"""

from snakes.nets import PetriNet
from snakes.nets import Place
from snakes.nets import Substitution
from snakes.nets import Transition
from snakes.nets import Variable
from hlpr_cadence.srv import *
import rospy

RESOURCES = ['floor']

class ResourceController():
  def __init__(self):
    self.petri_net_ = PetriNet('resource_controller')
    self.places_ = []
    self.transitions_ = []

    # Global places.
    self.places_.append(Place('free', RESOURCES))

    # Resource controller (user) places.
    self.places_.append(Place('requested_user', []))
    self.places_.append(Place('owned_user', []))

    # Resource controller (robot) places.
    self.places_.append(Place('requested_robot', []))
    self.places_.append(Place('owned_robot', []))

    # Global transitions.
    self.transitions_.append(Transition('yield'))
    self.transitions_.append(Transition('barge_in'))

    # Resource controller (user) transitions.
    self.transitions_.append(Transition('request_user'))
    self.transitions_.append(Transition('seize_user'))
    self.transitions_.append(Transition('release_user'))

    # Resource controller (robot) transitions.
    self.transitions_.append(Transition('request_robot'))
    self.transitions_.append(Transition('release_robot'))

    for place in self.places_:
      self.petri_net_.add_place(place)
    for transition in self.transitions_:
      self.petri_net_.add_transition(transition)

    # Arcs.
    self.petri_net_.add_input('free', 'seize_user', Variable('floor'))
    self.petri_net_.add_input('requested_user', 'seize_user', Variable("floor"))
    self.petri_net_.add_input('requested_user', 'yield', Variable("floor"))
    self.petri_net_.add_input('owned_user', 'barge_in', Variable("floor"))
    self.petri_net_.add_input('owned_user', 'release_user', Variable("floor"))
    self.petri_net_.add_input('requested_robot', 'barge_in', Variable("floor"))
    self.petri_net_.add_input('requested_robot', 'release_robot', Variable("floor"))
    self.petri_net_.add_input('owned_robot', 'yield', Variable("floor"))
    self.petri_net_.add_input('owned_robot', 'release_robot', Variable("floor"))
    self.petri_net_.add_output('free', 'release_user', Variable("floor"))
    self.petri_net_.add_output('free', 'release_robot', Variable("floor"))
    self.petri_net_.add_output('requested_user', 'request_user', Variable("floor"))
    self.petri_net_.add_output('owned_user', 'seize_user', Variable("floor"))
    self.petri_net_.add_output('owned_user', 'release_user', Variable("floor"))
    self.petri_net_.add_output('owned_user', 'yield', Variable("floor"))
    self.petri_net_.add_output('requested_robot', 'request_robot', Variable("floor"))
    self.petri_net_.add_output('owned_robot', 'barge_in', Variable("floor"))

  def get_marking(self):
    return self.petri_net_.get_marking()

  def has_place(self, name):
    return self.petri_net_.has_place(name)

  def has_transition(self, name):
    return self.petri_net_.has_transition(name)

  def place(self, name=None):
    return self.petri_net_.place(name)

  def transition(self, name=None):
    return self.petri_net_.transition(name)

  def AddTokenToPlace(self, place, token):
    if not self.petri_net_.has_place(place):
      raise ValueError("Does not have place: %s" % place)
    self.petri_net_.place(place).add(token)
    print("Resource marking after add: %s" % str(resource_controller.get_marking()))
    return True

  def RemoveTokenFromPlace(self, place, token):
    if not self.petri_net_.has_place(place):
      raise ValueError("Does not have place: %s" % place)
    place = self.petri_net_.place(place)
    if token not in place:
      return False
    place.remove(token)
    print("Resource marking after remove: %s" % str(resource_controller.get_marking()))
    return True

  # def Run(self):
  #   while not rospy.is_shutdown():
  #     for transition in self.transitions_:
  #       if len(transition.modes()) > 0:
  #         print("Markings before firing transition (%s): %s"
  #               % (transition.name, self.petri_net_.get_marking()))
  #         transition.fire(transition.modes().pop())
  #         print("Markings after firing transition (%s): %s"
  #               % (transition.name, self.petri_net_.get_marking()))

def handle_do_petri_net_arc(req):
  print "Received: (%s, %s, %s, %s)" \
      % (req.fire_guard, req.place, req.transition, req.token)
  if req.fire_guard == 'fire':
    if req.transition == 'add':
      resource_controller.AddTokenToPlace(req.place, req.token)
      return DoPetriNetArcResponse(True)
    if req.transition == 'remove':
      return DoPetriNetArcResponse(
          resource_controller.RemoveTokenFromPlace(req.place, req.token))
  if req.fire_guard == 'guard':
    if not resource_controller.has_place(req.place):
      raise rospy.ServiceException("Does not have place: %s" % req.place)
    place = resource_controller.place(req.place)
    print("Checking if resource token (%s) is in place (%s): %s"
          % (req.token, req.place, req.token in place))
    return DoPetriNetArcResponse(req.token in place)
  raise rospy.ServiceException("Invalid fire_guard input: %s" % req.fire_guard)

def main():
  global resource_controller
  resource_controller = ResourceController()
  print("Initial marking: %s" % str(resource_controller.get_marking()))

  rospy.init_node('do_petri_net_arc')
  s = rospy.Service('do_petri_net_arc', DoPetriNetArc, handle_do_petri_net_arc)
  print("Ready to do petri net arcs.")
  # resource_controller.Run()
  rospy.spin()

if __name__ == '__main__':
  main()
