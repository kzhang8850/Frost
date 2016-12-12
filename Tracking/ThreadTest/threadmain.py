import threading
import Queue
import threadtest1
import threadtest2
import sys
import os
import time


q = Queue.Queue()

thread1 = threadtest1.Chicken(q)
thread2 = threadtest2.Rooster(q)

thread1.start()
thread2.start()

kill = False


start_time = time.time()
while not kill:
    try:
        if not q.empty():
            data = q.get()
            elapsed_time = time.time() - start_time
            print elapsed_time
            start_time = time.time()
            # print data
    except KeyboardInterrupt:
        print "keyboard"
        kill = True
        thread1.kill = True
        thread2.kill = True

print "main loop closing"
