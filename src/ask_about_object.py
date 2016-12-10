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
from speak_actively import SpeakActively
from speak_passively import SpeakPassively
from point_at_object import PointAtObject
from look_at_object import LookAtObject
from wait_for_resource_free import WaitForResourceFree
from wait_for_resource_interrupted import WaitForResourceInterrupted
from look_center import LookCenter
import time
import random
import rospy

class AskAboutObject(Action):
	def __init__(self, object_name):
		self.object_name_ = object_name
		resource_name = 'object_' + object_name
		Action.__init__(self, 'ask_about_' + resource_name, [resource_name],
        {resource_name: True},
        {resource_name: True})
		self.active_ = rospy.get_param('active', True)
		self.wait_time_ = 0.25 if self.active_ else 1.0

	def Task(self):
		bowl_phrases = [
            " is full",
            " is blue",
            " can hold things",
            " can hold food",
            " can be full",
            " can hold soup"]
		banana_phrases = [
            " is yellow",
            " can be peeled",
            " is a vegetable",
            " is something I can eat",
            " is liked by monkeys",
            " is sour"]
		mug_phrases = [
            " is full",
            " is blue",
            " can hold tea",
            " can hold liquid",
            " has a handle",
            " might be hot"]

		tongs_phrases = [
            " can grab things",
            " are metal",
            " are plastic",
            " are purple",
            " are black and silver",
            " can grab food"]

		pitcher_phrases = [
            " can be poured",
            " is metal",
            " is black",
            " is blue",
            " can hold tea",
            " is plastic"]

		sugar_phrases = [
            " is sweet",
            " is salty",
            " is black",
            " is for dessert",
            " has grains",
            " is blue"]

		time.sleep(self.wait_time_)
		ActionProcess('', LookAtObject(self.object_name_)).Run()
		if self.active_:
			ActionProcess('',SpeakActively(150, 50,
          "I know some things about the " + self.object_name_)).Run()
		else:
			ActionProcess('',SpeakPassively(150, 50,
          "I know some things about the " + self.object_name_)).Run()
		time.sleep(self.wait_time_)

		phrases_used = []

		for i in range(3):
			random_phrase = random.randint(0,5)
			while(random_phrase in phrases_used):
				random_phrase = random.randint(0,5)
			phrases_used.append(random_phrase)
			phrase = ""
			if(self.object_name_ == 'bowl'):
				phrase = "The " + self.object_name_ + bowl_phrases[random_phrase]
			elif(self.object_name_ == 'banana'):
				phrase = "The " + self.object_name_ + banana_phrases[random_phrase]
			elif(self.object_name_ == 'mug'):
				phrase = "The " + self.object_name_ + mug_phrases[random_phrase]
			elif(self.object_name_ == 'tongs'):
				phrase = "The " + self.object_name_ + tongs_phrases[random_phrase]
			elif(self.object_name_ == 'pitcher'):
				phrase = "The " + self.object_name_ + pitcher_phrases[random_phrase]
			elif(self.object_name_ == 'sugar'):
				phrase = "The " + self.object_name_ + sugar_phrases[random_phrase]
			if self.active_:
				ActionProcess('', SpeakActively(150, 50, phrase)).Run()
			else:
				ActionProcess('', SpeakPassively(150, 50, phrase)).Run()
			time.sleep(self.wait_time_)

		ActionProcess('', LookCenter()).Run()
    # ActionProcess('',
    # WaitForResourceInterrupted('floor')).Run()
    # time.sleep(6)
    # ActionProcess('',
    # WaitForResourceFree('floor')).Run()

def main():
  ActionProcess('', AskAboutObject('mug')).Run()

if __name__ == '__main__':
  main()

