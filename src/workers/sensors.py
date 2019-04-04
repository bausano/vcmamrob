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
    print('Sensors message')
    points = []

    for point in message.points:
      x = round(math.fabs(point.x), 3)
      print(x)
      points.append(x)

    return points

