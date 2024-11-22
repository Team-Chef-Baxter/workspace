import cv2
import numpy as np

# Load YOLO model
net = cv2.dnn.readNet("yolov4.weights", "yolov4.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Load COCO names
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Initialize video capture (0 for webcam or provide a video file path)
video = cv2.VideoCapture(0)

# Initialize list for trackers and bboxes
trackers = []

while True:
    ret, frame = video.read()
    if not ret:
        break

    # Resize frame for YOLO input
    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), (0, 0, 0), swapRB=True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Process YOLO detections for the first frame or when no active trackers exist
    if len(trackers) == 0:
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:  # Adjust confidence threshold as needed
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Perform non-max suppression to remove overlapping boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Initialize KCF trackers for each detected object
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            tracker = cv2.TrackerKCF_create()
            tracker.init(frame, (x, y, w, h))
            trackers.append(tracker)

    # Update all trackers and draw the tracked bounding boxes
    for tracker in trackers:
        success, bbox = tracker.update(frame)
        if success:
            x, y, w, h = map(int, bbox)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            trackers.remove(tracker)

    # Display the resulting frame
    cv2.imshow("YOLO + KCF Tracker", frame)

    # Exit on pressing the ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release resources
video.release()
cv2.destroyAllWindows()
