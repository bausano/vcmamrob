import rospy
from workers.worker import Worker
from nav_msgs.msg import Odometry

class PositionWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(PositionWorker, self).__init__('/robotino_node/odom', Odometry)

  """
  " @inheritDoc
  """
  def handle(self, message):
    print('Position worker message')
    rospy.loginfo(message)

    return {
      'x': 0,
      'y': 0,
    }

