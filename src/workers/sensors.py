import rospy
from workers.worker import Worker
from sensor_msgs.msg import PointCloud

class SensorsWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(SensorsWorker, self).__init__('/robotino_node/distance_sensors', PointCloud)

  """
  " @inheritDoc
  """
  def handle(self, message):
    print('Sensors message')
    rospy.loginfo(message)

    return rospy.get_time()

