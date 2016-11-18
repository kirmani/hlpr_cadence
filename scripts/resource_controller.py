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
from snakes.nets import Transition
from snakes.nets import Variable
from hlpr_cadence.srv import *
import rospy

class ResourceController():
  def __init__(self):
    self.petri_net_ = PetriNet('resource_controller')

    # Global places.
    self.petri_net_.add_place(Place('free', ['floor']))
    self.petri_net_.add_transition(Transition('yield'))
    self.petri_net_.add_transition(Transition('barge_in'))

    # Resource controller (user) places.
    self.petri_net_.add_place(Place('requested_user', []))
    self.petri_net_.add_place(Place('owned_user', []))
    self.petri_net_.add_transition(Transition('request_user'))
    self.petri_net_.add_transition(Transition('seize_user'))
    self.petri_net_.add_transition(Transition('release_user'))

    # Resource controller (robot) places.
    self.petri_net_.add_place(Place('requested_robot', []))
    self.petri_net_.add_place(Place('owned_robot', []))
    self.petri_net_.add_transition(Transition('request_robot'))
    self.petri_net_.add_transition(Transition('seize_robot'))
    self.petri_net_.add_transition(Transition('release_robot'))

    # Arcs.
    self.petri_net_.add_input('free', 'seize_user', Variable("floor"))
    self.petri_net_.add_input('free', 'seize_robot', Variable("floor"))
    self.petri_net_.add_input('requested_user', 'seize_user', Variable("floor"))
    self.petri_net_.add_input('requested_user', 'yield', Variable("floor"))
    self.petri_net_.add_input('owned_user', 'barge_in', Variable("floor"))
    self.petri_net_.add_input('owned_user', 'release_user', Variable("floor"))
    self.petri_net_.add_input('requested_robot', 'seize_robot', Variable("floor"))
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
    self.petri_net_.add_output('owned_robot', 'seize_robot', Variable("floor"))
    self.petri_net_.add_output('owned_robot', 'barge_in', Variable("floor"))

resource_controller = ResourceController()

def handle_do_petri_net_arc(req):
  print "Received: (%s, %s, %s)" \
      % (req.fire_guard, req.transition, req.token)
  if req.fire_guard == 'fire':
    if not self.petri_net_.has_place(req.place):
      raise rospy.ServiceException("Does not have place: %s" % req.place)
    place = self.petri_net_.has_place(req.place)
    place.add(req.token)
    return DoPetriNetArcResponse(True)
  if req.fire_guard == 'guard':
    if not self.petri_net_.has_place(req.place):
      raise rospy.ServiceException("Does not have place: %s" % req.place)
    place = self.petri_net_.has_place(req.place)
    return DoPetriNetArcResponse(req.token in place)
  raise rospy.ServiceException("Invalid fire_guard input: %s" % req.fire_guard)

def main():
  rospy.init_node('do_petri_net_arc')
  s = rospy.Service('do_petri_net_arc', DoPetriNetArc, handle_do_petri_net_arc)
  print("Ready to do petri net arcs.")
  rospy.spin()

if __name__ == '__main__':
  main()
