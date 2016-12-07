#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Speak action.
"""

from action import Action
import subprocess

class SpeakActively(Action):
  def __init__(self, rate, pitch, text):
    Action.__init__(self, 'speech', ['floor_factor'], {'floor_factor': True}, {})
    self.rate_ = rate
    self.pitch_ = pitch
    self.text_ = text

  def Task(self):
    cmd = ['espeak', '-p', str(self.pitch_), '-s', str(self.rate_), '-v',
           'en', self.text_]
    proc = subprocess.Popen(cmd).wait()