import cv2
import numpy as np
from ultralytics import YOLO
from PIL import Image

# Load YOLOv11 model
model = YOLO("yolov10x.pt")  # Replace with your model path if you have a custom-trained model

# Function to detect objects using YOLOv11
def detect_objects(frame):
    # Run YOLOv11 inference
    results = model(frame)
    
    boxes = []
    confidences = []
    class_ids = []
    
    # Iterate over the detected objects
    for result in results:
        for obj in result.boxes.data:
            x1, y1, x2, y2, confidence, class_id = obj[:6]
            boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
            confidences.append(float(confidence))
            class_ids.append(int(class_id))
    
    return boxes, confidences, class_ids, results[0].names  # Retrieve the class names

# Capture frame and run detection
cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    boxes, confidences, class_ids, class_names = detect_objects(frame)

    # Draw bounding boxes
    for i in range(len(boxes)):
        x, y, w, h = boxes[i]
        label = class_names[class_ids[i]]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
