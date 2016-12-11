import Queue
import threadtest1
import threadtest2

q = Queue.Queue()

thread1 = threadtest1.Chicken(q)
thread1.daemon = True
thread1.start()

thread2 = threadtest2.Rooster(q)
thread2.daemon = True
thread2.start()

try:
    while 1:
        if not q.empty():
            data = q.get()
            print data
except KeyboardInterrupt:
    print "keyboard"


print "outside while loop"
print "main loop closing"
