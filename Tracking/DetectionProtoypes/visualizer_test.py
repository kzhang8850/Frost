#!/usr/bin/python

import web

urls = (
    '/', 'index'
)

class index:
    def GET(self):

        return "chicken"

    def stuff(self):
        while True:

            return "chicken"

if __name__ == "__main__":
    app = web.application(urls, globals())
    app.run()
                
