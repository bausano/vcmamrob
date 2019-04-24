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
    upper_bonds = 0.61
    lower_bonds = 0.25
    points = []
    for point in range(len(message.points)):
      # Round the distance reported to 2 decimals.
      x = math.sqrt(math.pow(message.points[point].x, 2) + math.pow(message.points[point].y, 2))

      # Calculates the approximity in range <0; 1>.
      distance = (x - lower_bonds) / (upper_bonds - lower_bonds)
      points.append(min(1, max(distance, 0)))

    return points

