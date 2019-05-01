import math
import rospy
from workers.worker import Worker
from sensor_msgs.msg import Image

class CameraWorker(Worker):

  """
  " Constructs new worker instance.
  """
  def __init__(self):
    super(CameraWorker, self).__init__('/image_raw', Image)

  """
  " @inheritDoc
  """
  def handle(self, message):
    image = self.bgr_to_luma(message)

    # Splits image in thirds. Count mow much black is in each part.
    thirds = [1, 1, 1, 1, 1, 1, 1, 1, 1]

    for y in range(len(image)):
      for x in range(message.width):
        index = int(math.floor(x / (message.width / 3)))

        thirds[min(index, 2)] += image[y][x]

    # Average pixel density.
    for third in range(3):
      thirds[third] = (thirds[third] / (message.width * message.height / 3)) / 255

    return thirds

  """
  " Converts raw byte BGR message to multi dimensional LUMA image.
  "
  " @param message ROS topic
  " @return Multi dimensional array of integers
  """
  def bgr_to_luma(self, message):
    image = []

    for pixel in range(0, len(message.data), 3):
      y = int(math.floor(pixel / (message.width * 3)))
      b = ord(message.data[pixel])
      r = ord(message.data[pixel + 1])
      g = ord(message.data[pixel + 2])

      if len(image) <= y:
        image.append([])

      image[y].append(
        round(0.2989 * r + 0.5870 * g + 0.1140 * b)
      )

    return image
