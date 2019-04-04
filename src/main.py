# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from core.loop import step
from geometry_msgs.msg import Twist
from workers.sensors import SensorsWorker
from workers.position import PositionWorker

# ROS Boot.
if __name__ == '__main__':
  print('🤖  VCMAMROB is booting!')

  # List of points the robot has to visit.
  targets = [
    { 'x': 5, 'y': 5 },
  ]

  # Starts a new node with a random hash appended to its name.
  rospy.init_node('vcmamrob', anonymous=True)

  print('✓ Node initialized.')

  # Constructs a new publisher instance that emits messages to the turtle
  robotino = rospy.Publisher('/robotino_node/cmd_vel', Twist, queue_size=50)

  print('✓ Publisher initialized.')

  # Booting all worker threads.
  workers = [
    SensorsWorker(),
  ]

  for worker in workers:
    worker.start()

  # Fetches the robotino position.
  positionWorker = PositionWorker()
  positionWorker.start()

  print('✓ Worker threads started.')

  # Constructs a new rate object on current thread that is used for publishing
  # frequency. There is at most one message published per tick.
  thread = rospy.Rate(10)

  # Loop that does not exit unless the core was shut down or all targets have
  # been visited.

  print('Entering the main loop.')

  while not rospy.is_shutdown() and len(targets) > 0:
    # Gets next velocity message.
    message = step(
      targets[len(targets) - 1],
      positionWorker.poll(),
      workers,
    )

    # Mimics frequency functionality.
    thread.sleep()

    # If the robot reached the target point, pops it out and goes to next one.
    if message == -1:
      print('✓ Target reached.')
      targets.pop(len(targets) - 1)
      continue

    # Sends message down the ROS topic for velocity.
    robotino.publish(message)

  print('All targets visited!')
