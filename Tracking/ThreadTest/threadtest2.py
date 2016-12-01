from threading import Thread

class Rooster(Thread):
    def __init__(self, queue, stop):
        super(Rooster, self).__init__()
        self.egg = 10
        self.queue = queue
        self.stop_event = stop

    def run(self):
        while not self.stop_event.is_set():
            print 'running'

            if self.egg > 0:
                self.queue.put((2, self.egg))
                self.egg -= 1
            else:
                self.egg = 10
