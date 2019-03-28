import rospy
from worker import Worker
from sensor_msgs.msg import PointCloud

class SensorsWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(SensorsWorker, self, '/robotino_node/distance_sensors', PointCloud).__init__()

  """
  " @inheritDoc
  """
  def handle(self, message):
    print('Handleing message')
    rospy.loginfo(message.data)

    return rospy.get_time()
