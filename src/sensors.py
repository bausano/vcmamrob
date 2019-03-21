import rospy
from worker import Worker
from sensor_msgs.msg import PointCloud

class SensorsWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(SensorsWorker, self, '', PointCloud).__init__()

  """
  " @inheritDoc
  """
  def handle(self, message):
    print('Handleing message')
    rospy.loginfo(message.data)

    return rospy.get_time()
