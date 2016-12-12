#!/usr/bin/python

#################################################################################################################################
"""
Raspi Visualizer Module - Acts as Frost's Visualization for Testing

An Attempt to convert our working code base onto a Raspberry Pi
Attempt to send data from Raspi to a server to be picked up for Visualization

##########UNFINISHED############3


************************************************************ARCHIVED*************************************************************
- Determined that the Raspberry Pi's computational power is not enough for all our algorithms, even under best conditions
still too slow
- The amount of hardware used by this project overloads the Raspberry Pi and causes too under voltage warnings from too much 
current draw. 

We stopped the exploration mainly due to this for fear of damaging the Raspberry Pi or lack of sufficient computatinal power
Stopped as of 12/9/16


Written by Kevin Zhang
"""
################################################################################################################################


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
                
