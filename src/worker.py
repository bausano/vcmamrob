import rospy
from threading import Event, Semaphore, Thread

# @abstract
class Worker(Thread):

  """
  " Constructs new worker instance.
  """
  def __init__(self, topic, messageType):
    super(Worker, self).__init__()

    # Rospy settings.
    self.topic = topic
    self.messageType = messageType

    # Event signalizing the worker should make a computation.
    self.ready = Event()
    self.ready.set()
    self.output = None
    # Prevents triggering the callback multiple times while a compute request is
    # set to true.
    self.inProgress = Event()

  """
  " Runs the worker.
  """
  def run(self):
    rospy.Subscriber(self.topic, self.messageType, self.listen)

  """
  " If asked by master, computes the movability circle.
  "
  " @param {PointCloud} Sensors message.
  """
  def listen(self, message):
    # If the master did not request new data, then ignore the message.
    if self.ready.isSet() or self.inProgress.isSet():
      return

    # Blocks other events.
    self.inProgress.set()

    # Computes the output
    self.output = self.handle(message.data)

    # Signalizes that the output is ready.
    self.ready.set()
    self.inProgress.clear()

  """
  " Waits for worker to report on the input status.
  """
  def poll(self):
    # Asks the worker to recompute output.
    self.ready.clear()
    # Waits for the worker to finish.
    self.ready.wait()

    return self.output

  """
  " Abstract handle method. Each worker implements the message parsing based
  " on the topic.
  "
  " @param {T} message
  """
  def handle(self, _):
    raise NotImplementedError('Worker::handle not implemented.')
