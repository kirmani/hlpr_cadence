#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""

"""

class PetriNetNode(object):
  def __init__(self, name):
    self.name = name
    self.outputs_ = []

  def AddOutput(self, node):
    self.outputs_.append(node)

  def GetOutputs(self):
    return self.outputs_

class PetriNetTransition(PetriNetNode):
  def fire(self):
    if self.activated():
      pass

  def activated(self):
    return True

class PetriNetPlace(PetriNetNode):
  def GetTransitions(self):
    return self.outputs_

class PetriNetToken(PetriNetNode):
  def __init__(self, name, location):
    PetriNetNode.__init__(self, name)
    self.location_ = location

  def GetLocation(self):
    return self.location_

  def SetLocation(self, location):
    self.location_ = location

