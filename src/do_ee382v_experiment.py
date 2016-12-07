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
from wait_for_resource_interrupted import WaitForResourceInterrupted
import random
import time

class DoEE382VExperiment(Action):
	def __init__(self):
		Action.__init__(self, 'do_ee382v_experiment', [], {}, {})
		self.objects_ = ['mug', 'banana', 'bowl']
		random.shuffle(self.objects_)

	def Task(self):
		print("wait a little bit, then say something to begin")
		#ActionProcess('',
        #WaitForResourceInterrupted('floor')).Run()
		time.sleep(5)
		for obj in self.objects_:
			ActionProcess('', AskAboutObject(obj)).Run()

def main():
  ActionProcess('', DoEE382VExperiment()).Run()

if __name__ == '__main__':
  main()
