from threading import Thread

class Chicken(Thread):
    def __init__(self, queue):
        super(Chicken, self).__init__()
        self.chick = 0
        self.queue = queue
        self.kill = False

    def run(self):
        while not self.kill:

            if self.chick < 10:
                self.queue.put((1, self.chick))
                self.chick += 1
            else:
                self.chick = 0

        print "Chicken out"


