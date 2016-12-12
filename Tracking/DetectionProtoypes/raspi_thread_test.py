from multiprocessing import Process
import time

class Chicken(Process):
    def __init__(self, queue):
        super(Chicken, self).__init__()
        self.chick = 0
        self.queue = queue

    def run(self):
        while True:

            if self.chick < 10:
                self.queue.put((1, self.chick))
                self.chick += 1
            else:
                self.chick = 0