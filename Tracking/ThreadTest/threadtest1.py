from multiprocessing import Process

class Chicken(Process):
    def __init__(self, queue):
        super(Chicken, self).__init__()
        self.chick = 0
        self.queue = queue

    def run(self):
        while 1:
            if self.chick < 10:
                self.queue.put((1, self.chick))
                self.chick += 1
            else:
                self.chick = 0
        print "Chicken out"
