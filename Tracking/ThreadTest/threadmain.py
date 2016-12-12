import Queue
import threadtest1
import threadtest2
import multiprocessing
import time

q = multiprocessing.Queue()

thread1 = threadtest1.Chicken(q)
thread1.start()

thread2 = threadtest2.Rooster(q)
thread2.start()

start_time = time.time()

try:
    while 1:
        if not q.empty():
            data = q.get()
            elapsed_time = time.time() - start_time
            print elapsed_time
            start_time = time.time()
            # print data
except KeyboardInterrupt:
    print "keyboard"
    thread1.terminate()
    thread2.terminate()


print "outside while loop"
print "main loop closing"
