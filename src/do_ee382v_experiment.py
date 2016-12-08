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
from ask_about_object import AskAboutObject
from look_center import LookCenter
from resource_controller import ResourceControllerApi
from wait_for_resource_interrupted import WaitForResourceInterrupted
import random
import time

def on_interrupt(action):
    global do_ee382v_experiment
    do_ee382v_experiment.SetInterrupted(True)
    ActionProcess('', LookCenter()).Run()

class DoEE382VExperiment(Action):
    def __init__(self):
        Action.__init__(self, 'do_ee382v_experiment', [], {}, {})
        self.objects_ = ['mug', 'banana', 'bowl']
        self.interrupted_ = False

    def Task(self):
        print("wait a little bit, then say something to begin")
        ActionProcess('',
            WaitForResourceInterrupted('floor')).Run()
        while len(self.objects_) > 0:
            obj = self.objects_[random.randint(0, len(self.objects_) - 1)]
            if ResourceControllerApi.CheckGuard('free', 'object_' + obj):
                ask_about_object = AskAboutObject(obj)
                ask_about_object.OnInterrupt(on_interrupt)
                self.interrupted = False
                ActionProcess('', ask_about_object).Run()
                if not self.interrupted_:
                    self.objects_.remove(obj)

    def SetInterrupted(self, interrupted):
        self.interrupted_ = interrupted

def main():
    global do_ee382v_experiment
    do_ee382v_experiment = DoEE382VExperiment()
    ActionProcess('', do_ee382v_experiment).Run()

if __name__ == '__main__':
  main()
