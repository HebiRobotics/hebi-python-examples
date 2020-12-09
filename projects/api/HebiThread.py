#!/usr/bin/env python

from time import time
import threading
import hebi
from hebi import arm as arm_api
import numpy as np

class HebiThread(threading.Thread):
  # Main Thread for everything HEBI

  def __init__(self):
    # Create Thread
    print("Starting the HEBI Thread")
    threading.Thread.__init__(self)
    self.abort_flag = False

    # Arm setup
    arm_family   = "Arm"
    module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
    hrdf_file    = "hrdf/A-2085-06.hrdf"
    gains_file   = "gains/A-2085-06.xml"

    # Create Arm object
    self.arm = arm_api.create([arm_family],
                        names=module_names,
                        hrdf_file=hrdf_file)
    self.arm.load_gains(gains_file)

    # Control Variables
    self.goal = arm_api.Goal(self.arm.size)
    # self.which_goal = "clear"
    self.waypoint_1 = np.asarray([0, 0, 0, 0, 0, 0], dtype=np.float64)
    self.waypoint_2 = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0], dtype=np.float64)

    self.start()


  def run(self):
    # Main control Loop
    while not self.abort_flag:
      self.arm.update()
      # if self.goal.waypoint_count > 0:
      #   print('setting the waypoint and then clearing it')
      #   self.arm.set_goal(self.goal)
      #   self.arm.send()
      #   self.goal = arm_api.Goal(self.arm.size)

      self.arm.send()


  def abort(self):
    self.abort_flag = True

  def set_waypoint_1(self):
    new_goal = arm_api.Goal(self.arm.size)
    new_goal.add_waypoint(t=3, position=self.waypoint_1)
    # self.arm.cancel_goal()
    self.arm.set_goal(new_goal)
    self.arm.send()
    print("Going to Waypoint 1.")

  def clear_waypoints(self):
    self.arm.cancel_goal()
    print("Cleared Waypoint")





