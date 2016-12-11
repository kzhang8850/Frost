#!/usr/bin/python

import web
import multiprocessing
import Queue
import raspi_thread_test


q = multiprocessing.Queue()
thread1 = raspi_thread_test.Chicken(q)
thread1.start()

xdata = None

urls = (
    '/', 'index'
)


class index:
    def GET(self):

        if not q.empty():
            xdata = q.get()
            return xdata
           
if __name__ == "__main__":

    app = web.application(urls, globals())
    print "hello"
    app.run()
    print "running"
                
