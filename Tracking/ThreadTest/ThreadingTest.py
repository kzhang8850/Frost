from threading import Thread

class Chicken(Thread):
    def __init__(self, queue, stop):
        super(Chicken, self).__init__()
        self.chick = 0
        self.queue = queue
        self.stop_event = stop

    def run(self):
        print 'running'
        while not self.stop_event.is_set():
            print 'testes'
    
