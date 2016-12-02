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
from ask_about_object import AskAboutObject

def main():
  ActionProcess('', AskAboutObject('ketchup')).Run()

if __name__ == '__main__':
  main()

