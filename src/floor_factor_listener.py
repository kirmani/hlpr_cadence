from resource_listener import ResourceListener

class FloorFactorListener(ResourceListener):
    def __init__(self,floor_listener):
        self.floor_listener_ = floor_listener
        self.floor_factor_ = 0.5
        self.name = "floor_factor"

    def Poll(self, actions):
	    # print("The floor factor is currently " + str(self.floor_listener_.robot_speaking_count_ / (self.floor_listener_.user_speaking_count_ + 1) ))
	    # print("The robot count is " + str(self.floor_listener_.robot_speaking_count_))
      robot_count = self.floor_listener_.robot_speaking_count_
      user_count = self.floor_listener_.user_speaking_count_ + 1
      ratio = float(robot_count) / float(user_count)
      print("robot vs. user count: %s / %s = %s" % (robot_count, user_count, ratio))
      return ratio < self.floor_factor_
