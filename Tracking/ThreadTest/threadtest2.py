from threading import Thread

class Rooster(Thread):
    def __init__(self, queue):
        super(Rooster, self).__init__()
        self.egg = 10
        self.queue = queue
        self.kill = False
        
    def run(self):
        while not self.kill:

            if self.egg > 0:
                self.queue.put((2, self.egg))
                self.egg -= 1
            else:
                self.egg = 10

        print "Rooster done"

 