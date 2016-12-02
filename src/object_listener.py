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

import fcntl
import os
import sys
import termios
import time

class ObjectListener(ResourceListener):
  def __init__(self, object_name):
    ResourceListener.__init__(self, 'object_' + object_name)
    self.object_name_ = object_name
    self.minimum_hold_time_ = 0.5 # seconds
    self.last_update_time_ = 0
    print("Listening for object: %s" % object_name)


  def Poll(self):
    print('polling for resource: %s' % self.object_name_)
    now = time.time()

    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
    c = None

    try:
      c = sys.stdin.read(1)
    except IOError:
      pass

    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

    if c != None:
      self.last_update_time_ = now

    is_free = now - self.last_update_time_ < self.minimum_hold_time_

    if is_free:
      print('object is free')

    return is_free
