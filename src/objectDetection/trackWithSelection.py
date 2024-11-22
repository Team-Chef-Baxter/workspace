import cv2

# Create a video capture object (0 for webcam or provide a video file path)
video = cv2.VideoCapture(0)

# Read the first frame
ret, frame = video.read()

# Define an initial bounding box (x, y, width, height)
bbox = cv2.selectROI("Frame", frame, False)
# BBOX COULD BE CHANGED!!!!!!!

# Initialize the KCF tracker
tracker = cv2.TrackerKCF_create()

# Initialize the tracker on the first frame and the selected bounding box
tracker.init(frame, bbox)

while True:
    # Read a new frame from the video
    ret, frame = video.read()
    if not ret:
        break

    # Update the tracker to get the new position of the bounding box
    success, bbox = tracker.update(frame)

    # Draw the bounding box if tracking is successful
    if success:
        x, y, w, h = map(int, bbox)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
    else:
        cv2.putText(frame, "Tracking failure detected", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow("KCF Tracker", frame)

    # Exit on pressing the ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release the video capture and close all windows
video.release()
cv2.destroyAllWindows()
