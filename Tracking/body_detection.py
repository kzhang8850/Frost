import numpy as np
import cv2
import freenect

from threading import Thread



class BodyThread(Thread):
	def __init__(self, q, stop):
		super(BodyThread, self).__init__()
		self.queue = q
		self.stop_event = stop
		self.bodies = BodyDetector()
		self.body_data = None

	def run(self):
		while not self.stop_event.is_set():
			self.body_data = self.bodies.find_bodies()
			if self.body_data is not None:
				self.queue.put((2, self.body_data))
			k = cv2.waitKey(30) & 0xff

			if k == 27:

				break
		self.bodies.shut_down()





class BodyDetector(object):
	def __init__(self):
		self.winStride = (4,4)
		self.padding = (16,16)
		self.scale = 1.1
		self.meanShift = False

		self.cam = None

		########TOGGLE THIS TO CHANGE VIDEO INPUT
		self.cam = cv2.VideoCapture()
		self.hog = cv2.HOGDescriptor()
		self.history = []
		self.people_ranges = []

		self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

	def get_video(self):
		array,_ = freenect.sync_get_video()
		array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
		return array


	def find_bodies(self):
		########TOGGLE THESE TWO LINES TO CHANGE VIDEO INPUT
		#ret, frame = self.cam.read()
		frame = self.get_video()

		frame = cv2.resize(frame, (320, 240))
		#frame = cv2.resize(frame, (600, 400))

		(rects, weights) = self.hog.detectMultiScale(frame, winStride=self.winStride,
		padding=self.padding, scale=self.scale, useMeanshiftGrouping=self.meanShift)

		self.people_ranges = self.draw_rectangles(rects,frame)

		if len(self.history) < 10:
			self.history.append(self.people_ranges)
		else:
			self.history.pop(0)
			self.history.append(self.people_ranges)
		# print history

		cv2.namedWindow('frame', 0)
		cv2.resizeWindow('frame', 320, 240)

		cv2.imshow('frame',frame)

		return self.people_ranges


	def draw_rectangles(self,rects, frame):
		if len(rects) == 0 and len(self.history) > 0 and any(len(item) > 0 for item in self.history):
			#print "i'm in history"
			for i in range(len(self.history)-1, -1, -1):
				if len(self.history[i]) > 0:
					hist = self.non_max_suppression_fast(self.history[i], 0.5)
					for (x, y, w, h) in hist:
						cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
					return hist
		else:
			rects = self.non_max_suppression_fast(rects, 0.5)
			for (x, y, w, h) in rects:
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
			return rects

	def shut_down(self):
		if self.cam is None:
			self.cam.release()
		cv2.destroyAllWindows()


	# Malisiewicz et al.
	def non_max_suppression_fast(self,boxes, overlapThresh):
		# if there are no boxes, return an empty list
		if len(boxes) == 0:
			return []

		# if the bounding boxes integers, convert them to floats --
		# this is important since we'll be doing a bunch of divisions
		if boxes.dtype.kind == "i":
			boxes = boxes.astype("float")

		# initialize the list of picked indexes
		pick = []

		# grab the coordinates of the bounding boxes
		x1 = boxes[:,0]
		y1 = boxes[:,1]
		x2 = boxes[:,2]
		y2 = boxes[:,3]

		# compute the area of the bounding boxes and sort the bounding
		# boxes by the bottom-right y-coordinate of the bounding box
		area = (x2 - x1 + 1) * (y2 - y1 + 1)
		idxs = np.argsort(y2)

		# keep looping while some indexes still remain in the indexes
		# list
		while len(idxs) > 0:
			# grab the last index in the indexes list and add the
			# index value to the list of picked indexes
			last = len(idxs) - 1
			i = idxs[last]
			pick.append(i)

			# find the largest (x, y) coordinates for the start of
			# the bounding box and the smallest (x, y) coordinates
			# for the end of the bounding box
			xx1 = np.maximum(x1[i], x1[idxs[:last]])
			yy1 = np.maximum(y1[i], y1[idxs[:last]])
			xx2 = np.minimum(x2[i], x2[idxs[:last]])
			yy2 = np.minimum(y2[i], y2[idxs[:last]])

			# compute the width and height of the bounding box
			w = np.maximum(0, xx2 - xx1 + 1)
			h = np.maximum(0, yy2 - yy1 + 1)

			# compute the ratio of overlap
			overlap = (w * h) / area[idxs[:last]]

			# delete all indexes from the index list that have
			idxs = np.delete(idxs, np.concatenate(([last],
				np.where(overlap > overlapThresh)[0])))

		# return only the bounding boxes that were picked using the
		# integer data type
		return boxes[pick].astype("int")





if __name__ == "__main__":


	Bodies = BodyDetector()

	while True:


		crowd = Bodies.find_bodies()

		k = cv2.waitKey(30) & 0xff

		if k == 27:

			break
	Bodies.shut_down()
