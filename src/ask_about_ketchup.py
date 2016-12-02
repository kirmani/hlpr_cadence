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
from wait_for_resource_free import WaitForResourceFree
from wait_for_resource_interrupted import WaitForResourceInterrupted

class AskAboutKetchup(Action):
  def __init__(self):
    Action.__init__(self, 'ask_about_ketchup', [], {}, {})

  def Task(self):
    ActionProcess('ask_about_ketchup_speak',
        Speak(150, 50, "please tell me about this ketchup")).Run()
    ActionProcess('ask_about_ketchup_wfri',
        WaitForResourceInterrupted('floor')).Run()
    ActionProcess('ask_about_ketchup_wfrf',
        WaitForResourceFree('floor')).Run()
