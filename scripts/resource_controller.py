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

from hlpr_cadence.srv import *
from petri_net import *
import rospy

kResources = ['floor']
kPlaces = ['free', 'requested_robot', 'owned_robot', 'requested_user',
           'owned_user']

class ResourceControllerApi:
  @staticmethod
  def RemoveResourceFromPlace(place, token):
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      return do_petri_net_arc('remove', place, token).response
    except rospy.ServiceException, e:
      print("Service call failed: %s" % e)
      return False

  @staticmethod
  def AddResourceToPlace(place, token):
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      return do_petri_net_arc('add', place, token).response
    except rospy.ServiceException, e:
      print("Service call failed: %s" % e)
      return False

  @staticmethod
  def CheckGuard(place, token):
    rospy.wait_for_service('do_petri_net_arc')
    try:
      do_petri_net_arc = rospy.ServiceProxy('do_petri_net_arc', DoPetriNetArc)
      return do_petri_net_arc('guard', place, token).response
    except rospy.ServiceException, e:
      print("Service call failed: %s" % e)
      return False

class ReleaseRobotTransition(PetriNetTransition):
  def __init__(self, requested_robot, owned_robot, free):
    PetriNetTransition.__init__(self, 'release_robot')
    self.requested_robot_ = requested_robot
    self.owned_robot_ = owned_robot
    self.free_ = free

  def fire(self):
    self.requested_robot_.RemoveToken('floor')
    self.owned_robot_.RemoveToken('floor')
    self.free_.AddToken('floor')

  def activated(self):
    return self.requested_robot_.HasToken('floor') \
       and self.owned_robot_.HasToken('floor')

class YieldTransition(PetriNetTransition):
  def __init__(self, requested_user, owned_robot, owned_user):
    PetriNetTransition.__init__(self, 'yield')
    self.requested_user_ = requested_user
    self.owned_robot_ = owned_robot
    self.owned_user_ = owned_user

  def fire(self):
    self.requested_user_.RemoveToken('floor')
    self.owned_robot_.RemoveToken('floor')
    self.owned_user_.AddToken('floor')

  def activated(self):
    return self.requested_user_.HasToken('floor') \
       and self.owned_robot_.HasToken('floor')

class ResourceController(PetriNet):
  def __init__(self):
    PetriNet.__init__(self, 'resource_controller')
    self.places_ = {}

    # Places.
    for place in kPlaces:
      self.places_[place] = PetriNetPlace(place)

    # Transitions.
    self.transitions_.append(
        ReleaseRobotTransition(self.places_['requested_robot'],
                               self.places_['owned_robot'],
                               self.places_['free']))
    self.transitions_.append(
        YieldTransition(self.places_['requested_user'],
                               self.places_['owned_robot'],
                               self.places_['owned_user']))

    for resource in kResources:
      self.places_['free'].AddToken(resource)

  def AddTokenToPlace(self, place, token):
    if place not in self.places_:
      raise ValueError("Does not have place: %s" % place)
    self.places_[place].AddToken(token)
    self.Run()
    print("Marking after adding token (%s) to place (%s): %s"
          % (token, place, str(resource_controller.GetMarking())))

  def HasTokenInPlace(self, place, token):
    if place not in self.places_:
      raise ValueError("Does not have place: %s" % place)
    return self.places_[place].HasToken(token)

  def RemoveTokenFromPlace(self, place, token):
    if place not in self.places_:
      raise ValueError("Does not have place: %s" % place)
    remove_successful = self.places_[place].RemoveToken(token)
    self.Run()
    return remove_successful

  def GetMarking(self):
    marking = {}
    for place in kPlaces:
      marking[place] = self.places_[place].GetTokens()
    return marking

def handle_do_petri_net_arc(req):
  # print "Received: (%s, %s, %s)" \
  #     % (req.action, req.place, req.token)
  if req.action == 'add':
    resource_controller.AddTokenToPlace(req.place, req.token)
    return DoPetriNetArcResponse(True)
  if req.action == 'remove':
    return DoPetriNetArcResponse(
        resource_controller.RemoveTokenFromPlace(req.place, req.token))
  if req.action == 'guard':
    response = resource_controller.HasTokenInPlace(req.place, req.token)
    # print("Checking if resource token (%s) is in place (%s): %s"
    #       % (req.token, req.place, response))
    return DoPetriNetArcResponse(response)
  raise rospy.ServiceException("Invalid action input: %s" % req.action)

def main():
  global resource_controller
  resource_controller = ResourceController()
  print("Initial marking: %s" % str(resource_controller.GetMarking()))

  rospy.init_node('do_petri_net_arc')
  s = rospy.Service('do_petri_net_arc', DoPetriNetArc, handle_do_petri_net_arc)
  print("Ready to do petri net arcs.")
  # resource_controller.Run()
  rospy.spin()

if __name__ == '__main__':
  main()
