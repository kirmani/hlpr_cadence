#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
The "hello world" example of interruptable actions.
"""

from action_process import ActionProcess
from speak import Speak

kRate = 150
kPitch = 50
kHello = "please don't interrupt me i really want to finish this sentence" \
    "okay good it worked"

def on_interrupt(action):
  speak = Speak(kRate, kPitch, kHello)
  speak.OnInterrupt(on_interrupt)
  action_process = ActionProcess('speech_action_process', speak)
  action_process.Run()

def main():
  speak = Speak(kRate, kPitch, kHello)
  speak.OnInterrupt(on_interrupt)
  action_process = ActionProcess('speech_action_process', speak)
  action_process.Run()

if __name__ == '__main__':
  main()

