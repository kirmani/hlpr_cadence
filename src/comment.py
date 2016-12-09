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
from look_center import LookCenter
from nod import Nod
import time
import random
import rospy

class Comment(Action):
    def __init__(self):
        Action.__init__(self, 'comment', [],{},{})
        self.active_ = True
        self.active_ = rospy.get_param('active', True)
        self.wait_time_ = 0.25 if self.active_ else 1.0

    def Task(self):
        comments = [
			"You are a good teacher", 
			"You look nice today", 
			"I like bananas",
			"My name is Gemini"]
        time.sleep(self.wait_time_)
        ActionProcess('', LookCenter()).Run()
        time.sleep(self.wait_time_)
        ActionProcess('', Nod()).Run()

        random_comment = random.randint(0,len(comments) - 1)
        if self.active_:
            ActionProcess('', SpeakActively(150, 50, comments[random_comment])).Run()
        else:
            ActionProcess('', SpeakPassively(150, 50, comments[random_comment])).Run()
        time.sleep(self.wait_time_)

        ActionProcess('', Nod()).Run()

def main():
  ActionProcess('', Comment()).Run()

if __name__ == '__main__':
  main()

