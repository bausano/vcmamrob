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
    image = bgr_to_luma(message)

    # TODO: Split image into thirds. Count mow much black is in each part.
    #       The darker, the lesser result. Match it with correct directions.

    return [1, 1, 1, 1, 1, 1, 1, 1, 1]

  def bgr_to_luma(message):
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