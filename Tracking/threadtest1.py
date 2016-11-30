from threading import Thread

class Chicken(Thread):
    def __init__(self, queue, stop):
        super(Chicken, self).__init__()
        self.chick = 0
        self.queue = queue
        self.stop_event = stop

    def run(self):
        print 'also running'
        while not self.stop_event.is_set():
            if self.chick < 10:
                self.queue.put((1, self.chick))
                self.chick += 1
            else:
                self.chick = 0
