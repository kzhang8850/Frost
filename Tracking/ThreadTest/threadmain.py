import Queue
import threadtest1
import threadtest2
import multiprocessing

q = multiprocessing.Queue()

thread1 = threadtest1.Chicken(q)
thread1.start()

thread2 = threadtest2.Rooster(q)
thread2.start()

try:
    while 1:
        if not q.empty():
            data = q.get()
            print data
except KeyboardInterrupt:
    print "keyboard"
    thread1.terminate()
    thread2.terminate()


print "outside while loop"
print "main loop closing"
