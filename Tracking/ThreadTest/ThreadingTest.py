from threading import Thread

class TestThread(Thread):
    def __init__(self, queue, stop, model, screen, view):
        super(TestThread, self).__init__()
        self.queue = queue
        self.stop_event = stop
        self.model = model
        self.screen = screen
        self.view = view
	def run(self):
		print 'running'
		while not self.stop_event.is_set():
			print('testes')