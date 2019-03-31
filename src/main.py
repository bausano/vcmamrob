#!/usr/bin/env python

import math
import rospy
from workers.sensors import SensorsWorker

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

PROXIMITY_THRESHHOLD = 2.00

def determine_angle_for_shortest_path(current, target):
  a = math.fabs(current.x - target.x)
  b = math.fabs(current.y - target.y)
  c = math.hypot(a, b)

  return math.cos(b/c)

def create_direction_circle(workers):
  circle = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

  for worker in workers:
    worker_circle = worker.poll()

    for direction in range(len(worker_circle)):
      if circle[direction] == -1:
        circle[direction] = worker_circle[direction]
        continue

      circle[direction] = min(circle[direction], worker_circle[direction])

  return circle

def get_circular_index(i):
  return 0 if i > 9 else 9 if i < 0 else i

def determine_direction(angle, circle):
  target = round(angle / (360 / len(circle) * math.pi / 180))

  for direction in range(math.ceil(len(circle) / 2) + 1):
    clockwise = circle[get_circular_index(target + direction)]
    anticlockwise = circle[get_circular_index(target - direction)]

    if (max(clockwise, anticlockwise) < PROXIMITY_THRESHHOLD):
      continue

    return target + direction if clockwise > anticlockwise else target - direction

  return circle.index(max(circle))
