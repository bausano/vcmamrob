#!/usr/bin/env python

from sensors import SensorsWorker

# ROS Boot.
if __name__ == '__main__':
  worker = SensorsWorker()

  worker.start()
