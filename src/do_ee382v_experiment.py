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
import random

class DoEE382VExperiment(Action):
  def __init__(self):
    Action.__init__(self, 'do_ee382v_experiment', [], {}, {})
    self.objects_ = ['ketchup', 'mustard', 'mayo']
    random.shuffle(self.objects_)

  def Task(self):
    for obj in self.objects_:
      ActionProcess('', AskAboutObject(obj)).Run()

def main():
  ActionProcess('', DoEE382VExperiment()).Run()

if __name__ == '__main__':
  main()
