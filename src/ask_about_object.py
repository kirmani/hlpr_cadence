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

class AskAboutObject(Action):
  def __init__(self, object_name):
    Action.__init__(self, 'ask_about_' + object_name, [], {}, {})
    self.object_name_ = object_name

  def Task(self):
    ActionProcess('',
        Speak(150, 50, "please tell me about the " + object_name)).Run()
    ActionProcess('',
        WaitForResourceInterrupted('floor')).Run()
    ActionProcess('',
        WaitForResourceFree('floor')).Run()
