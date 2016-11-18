import numpy as np
import cv2


# Malisiewicz et al.
def non_max_suppression_fast(boxes, overlapThresh):
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

    winStride = (4,4)
    padding = (16,16)
    scale = 1.03
    meanShift = False

    cap = cv2.VideoCapture(0)
    # fgbg = cv2.BackgroundSubtractorMOG()

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    while(1):

        ret, frame = cap.read()
        frame = cv2.resize(frame, (320, 240))


        (rects, weights) = hog.detectMultiScale(frame, winStride=winStride,
    	padding=padding, scale=scale, useMeanshiftGrouping=meanShift)

        rects = non_max_suppression_fast(rects, 0.4)

        for (x, y, w, h) in rects:
        	cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


        cv2.namedWindow('frame', 0)
        cv2.resizeWindow('frame', 320, 240)



        # fgmask = fgbg.apply(frame)



        cv2.imshow('frame',frame)

        k = cv2.waitKey(30) & 0xff

        if k == 27:

            break



    cap.release()
    cv2.destroyAllWindows()
