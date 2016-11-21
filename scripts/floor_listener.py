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

kInitialFloorHoldingThreshold = 0.001
kFormat = pyaudio.paInt16
kChannels = 2
kRate = 44100
kInputBlockTime = 0.05  # seconds
kInputFramesPerBlock = int(kRate * kInputBlockTime)
kShortNormalize = (1.0 / 32768.0)

# If we get this many noisy blocks in a row, increase the threshold.
kOversensitive = 15.0 / kInputBlockTime

# If we get this many quiet blocks in a row, decrease the threshold.
kUndersensitive = 120 / kInputBlockTime

# If the noise was longer than this many blocks, it's not a 'tap'.
kMaxTapBlocks = 0.15 / kInputBlockTime

class RequestUserTransition(PetriNetTransition):
  def __init__(self, listener):
    PetriNetTransition.__init__(self, 'request_user')
    self.listener_ = listener

  def fire(self):
    # Place resource tokens in requested place.
    print("Requesting resource for user: %s" % 'floor')
    ResourceControllerApi.AddResourceToPlace('requested_user', 'floor')

  def activated(self):
    return self.listener_.CheckFloor() and \
        not ResourceControllerApi.CheckGuard('owned_user', 'floor')

class SeizeUserTransition(PetriNetTransition):
  def __init__(self):
    PetriNetTransition.__init__(self, 'seize_user')

  def fire(self):
    print("Seizing resource for user: %s" % 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('requested_user', 'floor')
    ResourceControllerApi.RemoveResourceFromPlace('free', 'floor')
    ResourceControllerApi.AddResourceToPlace('owned_user', 'floor')

  def activated(self):
    return ResourceControllerApi.CheckGuard('requested_user', 'floor') and \
        ResourceControllerApi.CheckGuard('free', 'floor')

class FloorController(PetriNet):
  def __init__(self):
    PetriNet.__init__(self, 'floor_controller')
    floor_listener = FloorListener()

    self.transitions_.append(RequestUserTransition(floor_listener))
    self.transitions_.append(SeizeUserTransition())

  def EndCondition(self):
    return rospy.is_shutdown()

class FloorListener:
  def __init__(self):
    self.pa_ = pyaudio.PyAudio()
    self.stream_ = self.open_mic_stream()
    self.floor_holding_threshold_ = kInitialFloorHoldingThreshold
    self.noisy_count_ = kMaxTapBlocks + 1
    self.error_count_ = 0
    self.quiet_count_ = 0

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

  def CheckFloor(self):
    try:
      block = self.stream_.read(kInputFramesPerBlock)
    except IOError, e:
      # Damnit.
      self.error_count_ += 1
      print("(%d) Error recording: %s" % (self.error_count_, e))
      self.noisy_count_ = 1
      return

    amplitude = self.GetRms_(block)
    print(amplitude)
    return amplitude > self.floor_holding_threshold_
    #   # Noisy block.
    #   self.quiet_count_ = 0
    #   self.noisy_count_ += 1
    #   if self.noisy_count_ > kOversensitive:
    #     # Turn down the sensitivity.
    #     self.floor_holding_threshold_ *= 1.1
    # else:
    #   # Quiet block.
    #   if 1 <= self.noisy_count_ <= kMaxTapBlocks:
    #     self.FloorHoldingDetected()
    #   self.noisy_count_ = 0
    #   self.quiet_count_ += 1
    #   if self.quiet_count_ > kUndersensitive:
    #     # Turn up the sensitivity.
    #     self.floor_holding_threshold_ *= 0.9

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
  floor_controller = FloorController()
  floor_controller.Run()

if __name__ == '__main__':
  main()
