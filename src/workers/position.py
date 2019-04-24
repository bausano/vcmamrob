import rospy
from workers.worker import Worker
from nav_msgs.msg import Odometry

class PositionWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(PositionWorker, self).__init__('/odom', Odometry)

  """
  " @inheritDoc
  """
  def handle(self, message):
    return {
      'x': message.pose.pose.position.y,
      'y': message.pose.pose.position.x,
    }

