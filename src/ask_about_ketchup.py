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
from action_process import ActionProcess
from speak import Speak

class AskAboutKetchup(Action):
  def __init__(self):
    Action.__init__(self, 'ask_about_ketchup', [], {}, {})

  def Task(self):
    speak = Speak(150, 50, "please tell me about this ketchup")
    self.ask_about_ketchup_= ActionProcess(
            'ask_about_ketchup_action_process',
            speak)
    speak.OnInterrupt(self.ask_about_ketchup_.Run)
    self.ask_about_ketchup_.Run()
