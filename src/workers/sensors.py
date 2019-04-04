import math
import rospy
from workers.worker import Worker
from sensor_msgs.msg import PointCloud

class SensorsWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(SensorsWorker, self).__init__('/distance_sensors', PointCloud)

  """
  " @inheritDoc
  """
  def handle(self, message):
    # Callibrating the sensors with hard coded values cause time is money.
    upper_bonds = [0.61, 0.47, 0.11, 0.3, 0.57, 0.57, 0.31, 0.11, 0.47]
    lower_bonds = [0.29, 0.21, 0.05, 0.14, 0.26, 0.26, 0.14, 0.05, 0.22]
    points = []

    for point in range(len(message.points)):
      # Round the distance reported to 2 decimals.
      x = round(math.fabs(message.points[point].x), 2)

      # Calculates the approximity in range <0; 1>.
      points.append(
        (x - lower_bonds[point]) / (upper_bonds[point] - lower_bonds[point])
      )

    return points

