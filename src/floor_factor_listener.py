from resource_listener import ResourceListener
import rospy

class FloorFactorListener(ResourceListener):
    def __init__(self, floor_listener):
        self.name = "floor_factor"
        self.floor_listener_ = floor_listener
        self.active_ = rospy.get_param('active', True)
        print('initializing floor factor with parameter: %s' % self.active_)
        self.floor_factor_ = 2.0 if self.active_ else 0.5
        self.holding_ = True

    def Poll(self, actions):
	    # print("The floor factor is currently " + str(self.floor_listener_.robot_speaking_count_ / (self.floor_listener_.user_speaking_count_ + 1) ))
	    # print("The robot count is " + str(self.floor_listener_.robot_speaking_count_))
      robot_count = self.floor_listener_.robot_time_
      user_count = self.floor_listener_.user_time_ + 0.001
      if 'speech' not in actions:
        robot_count += 5.0
      ratio = float(robot_count) / float(user_count)
      # print("robot vs. user count: %s / %s = %s" % (robot_count, user_count, ratio))
      self.holding_ = ratio < self.floor_factor_
      return self.holding_
