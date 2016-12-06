#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
Point at object.
"""

from action import Action
import subprocess
import time



class PointAtObject(Action):
	def __init__(self, object_name):
		self.object_name_ = object_name
		resource_name = 'object_' + object_name
		Action.__init__(self, 'point_at_' + resource_name, [resource_name],
                {resource_name: True},
                {resource_name: True})

	def Task(self):
		"""TODO(taylor): Do point at object"""
		#time.sleep(4)
		cmd = ['python','../third_party/hlpr_hello_world/scripts/point.py',self.object_name_]
		proc = subprocess.Popen(cmd).wait()
		
		print("pointed at object: %s" % self.object_name_)
