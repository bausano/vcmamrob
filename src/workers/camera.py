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
    thirds = [0, 0, 0]

    for y in range(len(image)):
      for x in range(message.width):
        index = floor(x / (message.width / 3))

        thirds[index] = image[y][x]

    print(thirds)

    return [1, 1, 1, 1, 1, 1, 1, 1, 1]

  """
  " Converts raw byte BGR message to multi dimensional LUMA image.
  "
  " @param message ROS topic
  " @return Multi dimensional array of integers
  """
  def bgr_to_luma(self, message):
    image = []
    length = len(message.data) / 3

    for pixel in range(0, length, 3):
      y = int(math.floor(pixel / message.width))
      b = ord(message.data[pixel])
      r = ord(message.data[pixel + 1])
      g = ord(message.data[pixel + 2])

      if len(image) <= y:
        image.append([])

      image[y].append(
        round(0.2989 * r + 0.5870 * g + 0.1140 * b)
      )

    return image
