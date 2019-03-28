#!/usr/bin/env python

import rospy
from sensors import SensorsWorker

# ROS Boot.
if __name__ == '__main__':
  print("VCMAMROB Booting...")

  # Starts a new node with a random hash appended to its name.
  rospy.init_node('vcmamrob', anonymous=True)

  worker = SensorsWorker()

  worker.start()

  print(worker.poll())

  # Creates a direction object from the handle.
  # Creates a master object that decides which direction to go.
  # Load camera input. Get the camera input to direction object.
  # Master object takes both direction objects.
