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

PROXIMITY_THRESHOLD = 2.00
DESTINATION_THRESHOLD = 1.00

"""
" Finds the optimal angle x the robot should take to get to target point from
" the current one in an euclidian 2D space.
"
"   |\
" b | \
"   |  \ c
"   |__x\
"     a
"
" @param current Point the robot is currently in
" @param target Point it is trying to get to
" @return Float representing radians to turn by with respect of the x axis or
"         -1 if the target has been reached
"""
def determine_angle_for_shortest_path(current, target):
  a = math.fabs(current.x - target.x)
  b = math.fabs(current.y - target.y)
  c = math.hypot(a, b)

  return -1 if c < DESTINATION_THRESHOLD else math.cos(b/c)

"""
" Polls all workers to contribute to the direction circle that holds information
" about robot's environment in a circle split into 9 zones by 40 deg. Each zone
" is attributed by a float. The smaller the number, the closer is an obstacle in
" that direction. The consensus among the workers is reached by winner takes all
" strategy where the smaller number wins.
"
" @param workers Vector of workers contributing to the direction circle
" @return Vector of 9 floats each representing a zone in a circle
"""
def create_direction_circle(workers):
  if len(workers) == 0:
    raise Exception('At least one worker necessary to calculate the circle.')

  # Defaults the circle to a result of the first worker.
  circle = workers[0].poll()

  # For all other workers in the vector.
  for worker in range(len(workers) - 1):
    # Calculate their contribution.
    worker_circle = workers[worker].poll()

    # For all 9 directions in the worker_circle, replace the value in the
    # resulting circle if the worker reported the obstacle is closer than
    # the previous worker located it. Otherwise keep the current estimate.
    for direction in range(len(worker_circle)):
      # If the worker could not estimate given direction, skip the comparison.
      if worker_circle[direction] == -1:
        continue

      circle[direction] = min(circle[direction], worker_circle[direction])

  return circle

"""
" Circles indices around in range 0-9. Decrementing 0 gives 9, incrementing 9
" gives 0.
"
" @param i An index to be circularized
" @param circle_size The largest possible index before going back to 0
" @return A number in range <0; circle_size>
"""
def get_circular_index(i, circle_size):
  if i >= 0 and i <= circle_size:
    return i

  if i > circle_size:
    return get_circular_index(i - circle_size - 1, circle_size)

  return get_circular_index(circle_size + 1 + i, circle_size)

"""
" Finds the direction in given direction circle that is the closest to desired
" angle and does not have an obstacle.
"
" @param angle Ideal turn in radians
" @param circle Direction circle with obstacle proximity
" @return Number representing the best suited direction
"""
def determine_direction(angle, circle):
  # Calculates which zone in a circle approximates the angle the best.
  target = round(angle / (360 / len(circle) * math.pi / 180))
  circle_size = len(circle)

  # Our search space is number of directions. Since it is a circle, with each
  # iteration we go in both directions. An example of the search for 9
  # directions where the target is 7.
  #
  # i | directions checked
  # 0 | 7 7
  # 1 | 6 8
  # 2 | 5 9
  # 3 | 4 0
  # 4 | 3 1
  # 5 | 2 2
  #
  for direction in range(math.ceil(circle_size / 2) + 1):
    # Calculates changes in neighboring directions
    clockwise_index = get_circular_index(target + direction, circle_size)
    clockwise = circle[clockwise_index]

    anticlockwise_index = get_circular_index(target - direction, circle_size)
    anticlockwise = circle[clockwise_index]

    # If neither is larger than the minimum threshold (both have obstacles), go
    # to next iteration.
    if (max(clockwise, anticlockwise) < PROXIMITY_THRESHOLD):
      continue

    # Returns direction that has the least obstacles in the way.
    return clockwise_index if clockwise > anticlockwise else anticlockwise_index

  # TODO: Should stop and notify user about no suitable direction.
  return circle.index(max(circle))
