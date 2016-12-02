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
from point_at_object import PointAtObject

def main():
  point_at_object_action = PointAtObject('ketchup')
  action_process = ActionProcess('point_at_ketchup_action_process',
      point_at_object_action)
  action_process.Run()

if __name__ == '__main__':
  main()

