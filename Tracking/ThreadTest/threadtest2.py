from multiprocessing import Process

class Rooster(Process):
    def __init__(self, queue):
        super(Rooster, self).__init__()
        self.egg = 10
        self.queue = queue
        
    def run(self):
        while 1:
            if self.egg > 0:
                self.queue.put((2, self.egg))
                self.egg -= 1
            else:
                self.egg = 10

        print "Rooster done"