#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Object listener.
"""
from resource_listener import ResourceListener

class ObjectListener(ResourceListener):
  def __init__(self, object_name):
    ResourceListener.__init__(self, 'object_' + object_name)
    self.object_name_ = object_name
    print("Listening for object: %s" % object_name)

  def Poll(self, actions):
    "TODO(taylor): make this return True and False correctly"
    return True
