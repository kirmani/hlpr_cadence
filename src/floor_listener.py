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

# from petri_net import *
# from resource_controller import ResourceControllerApi
from resource_listener import ResourceListener
from sampler import Sampler

import math
import pyaudio
import struct

# For keyboard listener.
import termios, fcntl, sys, os, time

import random

# Dummy listener for debugging.
kUseKeyboardListener = False

kVerbose = True

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
    # constants
    ResourceListener.__init__(self, 'floor')
    self.format_ = pyaudio.paInt16
    self.channels_ = 2
    self.rate_ = 44100
    input_block_time = 0.05  # seconds
    self.input_frames_per_block_ = int(self.rate_ * input_block_time)
    self.short_normalize_ = (1.0 / 32768.0)

    self.pa_ = pyaudio.PyAudio()
    self.stream_ = self.open_mic_stream()
    self.floor_holding_threshold_ = 0.01
    self.error_count_ = 0
    self.minimum_hold_time_ = 0.5 # seconds
    self.last_update_time_ = 0
    self.samplers_ = {}
    # self.expected_volume_sampler_ = Sampler()

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
    stream = self.pa_.open(format=self.format_,
                          channels=self.channels_,
                          rate=self.rate_,
                          input=True,
                          input_device_index=device_index,
                          frames_per_buffer=self.input_frames_per_block_)
    return stream

  def Poll(self, actions):
    "Returns True if the floor is free"
    try:
      block = self.stream_.read(self.input_frames_per_block_)
    except IOError, e:
      # Damnit.
      # self.error_count_ += 1
      print("(%d) Error recording: %s" % (self.error_count_, e))
      return

    actions_hash = str(actions)
    if actions_hash not in self.samplers_:
      self.samplers_[actions_hash] = Sampler()
    now = time.time()

    # amplitude = random.uniform(0, 1)
    amplitude = self.GetRms_(block)
    self.samplers_[actions_hash].Sample(amplitude)

    return self.samplers_[actions_hash].IsConfidentValue(amplitude) == 0

  def GetRms_(self, block):
    # We will get one short out for each two chars in the string.
    count = len(block) / 2
    format = "%dh" % count
    shorts = struct.unpack(format, block)

    # Iterate over the block.
    sum_squares = 0.0
    for sample in shorts:
      # Sample is a signed short in +/- 32768. Normalize it to 1.0.
      n = sample * self.short_normalize_
      sum_squares += n * n

    return math.sqrt(sum_squares / count)
