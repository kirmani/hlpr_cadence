#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2016 kirmani <sean@kirmani.io>
#
# Distributed under terms of the MIT license.

"""
PyAudio Test
"""

# from petri_net import *
# from resource_controller import ResourceControllerApi
from collections import deque
from resource_listener import ResourceListener
from sampler import Sampler
import json
import math
import os
import rospy
import pyaudio
import struct
import time

kVerbose = True
kSensesFile = os.path.join(os.path.dirname(__file__),
            'data/senses.json')

class FloorListener(ResourceListener):
    def __init__(self):
        self.active_ = rospy.get_param('active', True)
        minimum_block_time = 0.05
        self.holding_ = False
        self.robot_speaking_count_ = 0
        self.user_speaking_count_ = 0
        self.response_delay_ = 1.0
        self.lapse_tolerance_ = 0.5 if self.active_ else 4.0
        self.lapse_deque_ = deque()
        self.conflict_tolerance_ = 1.0
        self.conflict_deque_ = deque()
        self.last_time_without_lapse_ = 0
        self.last_time_without_conflict_ = 0

        # constants
        ResourceListener.__init__(self, 'floor')
        self.format_ = pyaudio.paInt16
        self.channels_ = 2
        self.rate_ = 44100
        self.input_frames_per_block_ = int(self.rate_ * minimum_block_time)
        self.short_normalize_ = (1.0 / 32768.0)

        self.pa_ = pyaudio.PyAudio()
        self.stream_ = self.open_mic_stream()
        self.floor_holding_threshold_ = 0.01
        self.error_count_ = 0
        self.last_update_time_ = 0
        self.samplers_ = self.LoadSenses_()

    def LoadSenses_(self):
        try:
            with open(kSensesFile, 'r+') as f:
                senses_file = json.load(fp=f)
            samplers = {}
            for actions_hash in senses_file:
                save_data = senses_file[actions_hash]
                samplers[actions_hash] = Sampler()
                samplers[actions_hash].LoadFromSaveData(save_data)
            return samplers
        except IOError:
            print("Error loading senses file.")
            return {}

    def find_input_device(self):
        device_index = None
        for i in range(self.pa_.get_device_count()):
            devinfo = self.pa_.get_device_info_by_index(i)
            for keyword in ['mic', 'input']:
                if keyword in devinfo['name'].lower():
                    print("Found an input: device %d - %s" % (i, devinfo['name']))
                    device_index = i
                    return device_index
        if device_index == None:
            print("No preferred input found. Using default input device.")
        return device_index

    def open_mic_stream(self):
        device_index = self.find_input_device()
        stream = self.pa_.open(format=self.format_,
                          channels=self.channels_,
                          rate=self.rate_,
                          input=True,
                          input_device_index=device_index,
                          frames_per_buffer=self.input_frames_per_block_)
        return stream

    def Poll(self, actions):
        "Returns True if the floor is free"
        try:
            if self.stream_.get_read_available() < self.input_frames_per_block_:
                return not self.holding_
            block = self.stream_.read(self.input_frames_per_block_)
        except IOError, e:
            # Damnit.
            self.error_count_ += 1
            print("(%d) Error recording: %s" % (self.error_count_, e))
            return

        # actions_hash = ', '.join(actions)
        actions_hash = '' if 'speech' not in actions else 'speech'
        if actions_hash not in self.samplers_:
            self.samplers_[actions_hash] = Sampler()
        now = time.time()

        amplitude = self.GetRms_(block)
        self.samplers_[actions_hash].Sample(amplitude)

        self.conflict_deque_.append((amplitude, now))
        while now - self.conflict_deque_[0][1] > self.conflict_tolerance_:
            self.conflict_deque_.popleft()
        conflict_threshold_volume = self.samplers_[actions_hash].expectation_ \
                + math.sqrt(self.samplers_[actions_hash].variance_)
        num_above_threshold = 0
        for sample in self.conflict_deque_:
            if sample[0] > conflict_threshold_volume:
                num_above_threshold += 1
        conflict_confidence = float(num_above_threshold) \
                / float(len(self.conflict_deque_))
        # print("conflict confidence: %s" % conflict_confidence)

        self.lapse_deque_.append((amplitude, now))
        while now - self.lapse_deque_[0][1] > self.lapse_tolerance_:
            self.lapse_deque_.popleft()
        lapse_threshold_volume = self.samplers_[actions_hash].expectation_ \
                + math.sqrt(self.samplers_[actions_hash].variance_)
        num_below_threshold = 0
        for sample in self.lapse_deque_:
            if sample[0] < lapse_threshold_volume:
                num_below_threshold += 1
        lapse_confidence = float(num_below_threshold) \
                / float(len(self.lapse_deque_))
        # print("lapse confidence: %s" % lapse_confidence)

        if lapse_confidence < 0.5:
            self.last_time_without_lapse_ = now

        if conflict_confidence < 0.5:
            self.last_time_without_conflict_ = now

        # Check if we got a valid most recent value.
        # if self.samplers_[actions_hash].IsConfidentValue(amplitude) == 0:
        #     self.last_update_time_ = now
        # self.holding_ = now - self.last_update_time_ > self.response_delay_

        if now - self.last_update_time_ < self.response_delay_:
            return not self.holding_

        if self.holding_:
            # User has the floor.
            self.user_speaking_count_ += 1
            if now - self.last_time_without_lapse_ > self.lapse_tolerance_:
                # Lapsed long enough. User yields the floor.
                self.holding_ = False
                self.last_update_time_ = now
        else:
            # User does not have the floor.
            if 'speech' in actions:
                # Robot has the floor.
                self.robot_speaking_count_ += 1
                if now - self.last_time_without_conflict_ > \
                        self.conflict_tolerance_:
                    # Conflicted long enough. Robot yields the floor.
                    self.holding_ = True
                    self.last_known_hold_time_ = now
            else:
                # No one has the floor. Check if user is trying to take the
                # floor.
                if conflict_confidence > 0.5:
                    self.holding_ = True
        return not self.holding_

    def GetRms_(self, block):
        # We will get one short out for each two chars in the string.
        count = len(block) / 2
        format = "%dh" % count
        shorts = struct.unpack(format, block)

        # Iterate over the block.
        sum_squares = 0.0
        for sample in shorts:
            # Sample is a signed short in +/- 32768. Normalize it to 1.0.
            n = sample * self.short_normalize_
            sum_squares += n * n

        return math.sqrt(sum_squares / count)

    def OnShutdown(self):
        save_data = {}
        for actions_hash in self.samplers_:
            save_data[actions_hash] = self.samplers_[actions_hash].GetSaveData()
            self.WriteSenses_(save_data)
        if kVerbose:
            print("saving to long term memory")
            print(save_data)

    def WriteSenses_(self, data):
        j = json.dumps(data, indent=4)
        with open(kSensesFile, 'w') as f:
            f.write(j)
