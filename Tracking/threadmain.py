import threading
import Queue
import threadtest1
import threadtest2
import sys
import os


q = Queue.Queue()

thread1_stop = threading.Event()
thread1 = threadtest1.Chicken(q, thread1_stop)
thread2_stop = threading.Event()
thread2 = threadtest2.Rooster(q, thread2_stop)

thread1.setDaemon = True
thread2.setDaemon = True

thread1.start()
thread2.start()

while True:
    try:
        if not q.empty():
            data = q.get()
            print data
    except KeyboardInterrupt:
        print "keyboard"
        thread1_stop.set()
        thread2_stop.set()
        break
os._exit()
