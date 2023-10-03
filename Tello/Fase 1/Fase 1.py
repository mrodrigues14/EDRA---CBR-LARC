from djitellopy import Tello
import cv2
import time
import threading
import numpy as np

# Load YOLO model
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

def streaming():
    tello.streamon()
    while True:
        img = tello.get_frame_read().frame

        # YOLO detection
        height, width, channels = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Information to show on the screen
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Check if base is detected and adjust drone's position
        for i in range(len(boxes)):
            if i in indexes:
                label = str(classes[class_ids[i]])
                if label == "base":
                    base_center_x = boxes[i][0] + boxes[i][2] // 2
                    base_center_y = boxes[i][1] + boxes[i][3] // 2
                    img_center_x = width // 2
                    img_center_y = height // 2

                    threshold = 50  # Adjust as needed

                    if abs(base_center_x - img_center_x) > threshold:
                        if base_center_x < img_center_x:
                            tello.move_left(20)
                        else:
                            tello.move_right(20)

                    if abs(base_center_y - img_center_y) > threshold:
                        if base_center_y < img_center_y:
                            tello.move_up(20)
                        else:
                            tello.move_down(20)

                    # If base is centered, land the drone
                    if abs(base_center_x - img_center_x) <= threshold and abs(base_center_y - img_center_y) <= threshold:
                        tello.land()
                        return

                # Draw bounding boxes for all detected objects
                color = (0, 255, 0)
                cv2.rectangle(img, (boxes[i][0], boxes[i][1]), (boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3]), color, 2)
                cv2.putText(img, label + " " + str(round(confidences[i], 2)), (boxes[i][0], boxes[i][1] - 10), cv2.FONT_HERSHEY_PLAIN, 1, color, 2)

        cv2.imshow("frame", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    streaming_thread.join()
