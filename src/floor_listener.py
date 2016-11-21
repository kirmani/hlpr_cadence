#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
PyAudio Test
"""

from petri_net import *
from resource_controller import ResourceControllerApi

import math
import pyaudio
import rospy
import struct

# For keyboard listener.
import termios, fcntl, sys, os, time

# Dummy listener for debugging.
kUseKeyboardListener = False

kVerbose = True

kInitialFloorHoldingThreshold = 0.01
kFormat = pyaudio.paInt16
kChannels = 2
kRate = 44100
kInputBlockTime = 0.05  # seconds
kInputFramesPerBlock = int(kRate * kInputBlockTime)
kShortNormalize = (1.0 / 32768.0)

class RequestUserTransition(PetriNetTransition):
  def __init__(self, listener):
    PetriNetTransition.__init__(self, 'request_user')
    self.listener_ = listener

  def fire(self):
    # Place resource tokens in requested place.
    if kVerbose: print("Requesting resource for user: %s" % 'floor')
    ResourceControllerApi.AddResourceToPlace('requested_user', 'floor')

  def activated(self):
    return self.listener_.Poll() and \
        not ResourceControllerApi.CheckGuard('owned_user', 'floor')

class SeizeUserTransition(PetriNetTransition):
  def __init__(self):
    PetriNetTransition.__init__(self, 'seize_user')

  def fire(self):
    if kVerbose: print("Seizing resource for user: %s" % 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('requested_user', 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('free', 'floor')
    ResourceControllerApi.AddResourceToPlace('owned_user', 'floor')

  def activated(self):
    return ResourceControllerApi.CheckGuard('requested_user', 'floor') and \
        ResourceControllerApi.CheckGuard('free', 'floor')

class ReleaseUserTransition(PetriNetTransition):
  def __init__(self, listener):
    PetriNetTransition.__init__(self, 'release_user')
    self.listener_ = listener

  def fire(self):
    if kVerbose: print("Releasing resource for user: %s" % 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('owned_user', 'floor')
    ResourceControllerApi.AddResourceToPlace('free', 'floor')

  def activated(self):
    return not self.listener_.Poll() and \
        ResourceControllerApi.CheckGuard('owned_user', 'floor')

class ResourceListenerController(PetriNet):
  def __init__(self, listener):
    PetriNet.__init__(self, 'floor_controller')
    self.listener_ = listener

    self.transitions_.append(RequestUserTransition(self.listener_))
    self.transitions_.append(SeizeUserTransition())
    self.transitions_.append(ReleaseUserTransition(self.listener_))

  def Run(self):
    self.listener_.StartListening()
    PetriNet.Run(self)

  def EndCondition(self):
    return rospy.is_shutdown()

class ResourceListener:
  def StartListening(self):
    pass

  def Poll(self):
    return False

class KeyboardListener(ResourceListener):
  def StartListening(self):
    self.minimum_hold_time_ = 0.5 # seconds
    self.last_update_time_ = 0
    print("Listening for *any* keyboard input.")

  def Poll(self):
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

    return now - self.last_update_time_ < self.minimum_hold_time_

class FloorListener(ResourceListener):
  def __init__(self):
    self.pa_ = pyaudio.PyAudio()
    self.stream_ = self.open_mic_stream()
    self.floor_holding_threshold_ = kInitialFloorHoldingThreshold
    self.error_count_ = 0
    self.minimum_hold_time_ = 0.5 # seconds
    self.last_update_time_ = 0

  def find_input_device(self):
    device_index = None
    for i in range(self.pa_.get_device_count()):
      devinfo = self.pa_.get_device_info_by_index(i)
      for keyword in ['mic', 'input']:
        if keyword in devinfo['name'].lower():
          print("Found an input: device %d - %s" % (i, devinfo['name']))
          device_index = i
          return device_index
    if device_index == None:
      print("No preferred input found. Using default input device.")
    return device_index

  def open_mic_stream(self):
    device_index = self.find_input_device()
    stream = self.pa_.open(format=kFormat,
                          channels=kChannels,
                          rate=kRate,
                          input=True,
                          input_device_index=device_index,
                          frames_per_buffer=kInputFramesPerBlock)
    return stream

  def Poll(self):
    try:
      block = self.stream_.read(kInputFramesPerBlock)
    except IOError, e:
      # Damnit.
      self.error_count_ += 1
      print("(%d) Error recording: %s" % (self.error_count_, e))
      return

    now = time.time()

    amplitude = self.GetRms_(block)
    if amplitude > self.floor_holding_threshold_:
      self.last_update_time_ = now

    return now - self.last_update_time_ < self.minimum_hold_time_

  def GetRms_(self, block):
    # We will get one short out for each two chars in the string.
    count = len(block) / 2
    format = "%dh" % count
    shorts = struct.unpack(format, block)

    # Iterate over the block.
    sum_squares = 0.0
    for sample in shorts:
      # Sample is a signed short in +/- 32768. Normalize it to 1.0.
      n = sample * kShortNormalize
      sum_squares += n * n

    return math.sqrt(sum_squares / count)

def main():
  global args
  floor_listener = KeyboardListener() if kUseKeyboardListener else FloorListener()
  floor_listener_controller = ResourceListenerController(floor_listener)
  floor_listener_controller.Run()

if __name__ == '__main__':
  main()
