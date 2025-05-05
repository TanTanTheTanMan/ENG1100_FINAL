import cv2
import serial
import struct
import time
import math
import numpy as np

PORT = 'COM5' ## Change to camera port
BAUD = 1000000 ## camera stream 
SER_TIMEOUT = 2 ## incase of no data from camera
MAX_COMMAND_RATE = 1.0

ser = serial.Serial(PORT, baudrate=BAUD, timeout=SER_TIMEOUT) ## Serial port for ESP32
last_sent_time = 0
last_detected = None

print("Starting Cube Detector...")
## looks at 
def find_distance(p1, p2): 
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def send_command(cmd): ## sends command to ESP32. checks if time since last command is greater than MAX_COMMAND_RATE before sending to avoid flooding the ESP32.
    global last_sent_time
    now = time.time()
    if now - last_sent_time >= MAX_COMMAND_RATE:
        ser.write((cmd.strip() + "\n").encode('utf-8'))  ## send the command to the ESP32
        print("Sent to ESP32:", cmd) ## print the command sent to the ESP32 for debugging
        last_sent_time = now ## update the last sent time to the current time
## this is the function that reads the camera stream
def read_frame(): 
    retries = 3 
    while retries > 0:
        ser.reset_input_buffer() ## clear the input buffer to avoid reading old data
        time.sleep(0.15) ## give the camera time to start streaming
        ser.write(b"stream\n") ## send the command to start the camera stream

        start_time = time.time() ## start the timer to avoid infinite loop in case of no data from camera
        while True:
            if time.time() - start_time > 3: ## if the time exceeds 3 seconds, break the loop and flush the serial buffer
                print("Timeout waiting for frame header. Flushing serial...") ## print a message for debugging
                ser.reset_input_buffer()
                return None
            if ser.read(1) == b'\xFF': ## Start of JPEG header, sent TWICE becasue it worked better 
                if ser.read(1) == b'\xFF':
                    break

        length_bytes = ser.read(4) ## read the length of the JPEG data, little endian
        if len(length_bytes) < 4:
            retries -= 1
            time.sleep(1)
            continue

        frame_len = struct.unpack('<I', length_bytes)[0] ## unpack the length of the JPEG data from bytes to integer

        if frame_len <= 0 or frame_len > 150000: ## check if the length is valid
            retries -= 1
            time.sleep(1)
            continue

        jpeg_data = ser.read(frame_len) ## read the JPEG data
        if len(jpeg_data) != frame_len: ## check if the length of the data is valid
            retries -= 1
            time.sleep(1)
            continue

        arr = np.frombuffer(jpeg_data, dtype=np.uint8) ## convert the data to a numpy array
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR) ## decode the JPEG data to an image
        return img

    print("Failed to get valid frame after retries") ## if it fails to get a valid frame after retries, return None just to be safe
    return None

color_ranges = {
    ##"RED1": ([0, 50, 50], [4, 255, 255]),   READD IF NEEDED. HSV bounds for colors 
    "RED": ([170, 60, 60], [179, 255, 255]),
    "ORANGE": ([0, 100, 100], [20, 255, 255]),
    "GREEN": ([52, 80, 35], [90, 255, 255]),
    "BLUE": ([100, 150, 50], [130, 255, 255]),
    "YELLOW": ([20, 50, 90], [50, 255, 255])
}

pending_detection = None
CUBE_SIZES = [1.75, 2.0, 2.25] ## cube sizes in inches
ROI = (300, 75, 770, 550) ## cut camera image to this area for detection

cv2.namedWindow("Frame", cv2.WINDOW_NORMAL) ## create windows for the camera stream and mask. actually a physical mask 
cv2.namedWindow("Mask", cv2.WINDOW_NORMAL) ## for the mask of the detected color. displys the canny edges of the cube. useful for HSV tuning 

while True:
    frame = read_frame()
    if frame is None:
        print("No frame received, sending no cube...") ## incase of no data from camera so it doenst send garbage data to ESP32
        send_command("no cube") 
        pending_detection = None
        time.sleep(1.0)
        continue

    x1, y1, x2, y2 = ROI  ## cut the image to the ROI for detection. this is the area where the cube will be detected. ignore the rest of the image
    roi = frame[y1:y2, x1:x2].copy()
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) ## convert the image to HSV color space for easier color detection

    masks = {} 
    for label, (lower, upper) in color_ranges.items(): ## loop through the colors and create a mask for each color
        lower = np.array(lower, dtype=np.uint8) ## convert the lower and upper bounds to uint8 for OpenCV
        upper = np.array(upper, dtype=np.uint8)
        masks[label] = cv2.inRange(hsv, lower, upper) ## create a mask for the color in the image. 1 bit image where the color is white and the rest is black for more accurate detection


    mask_total = np.zeros_like(hsv[:, :, 0])  ## combine masks 
    detected_color = "UNKNOWN" ## default color if no color is detected, false negitive is better than false positive
    best_mask = None
    max_pixels = 0

    for label, mask in masks.items(): ## loop through the masks and find the one with the most pixels
        count = cv2.countNonZero(mask) ## count the number of white pixels in the mask
        if count > max_pixels: 
            max_pixels = count
            detected_color = label.replace("1", "").replace("2", "") ## remove the 1 and 2 from the color name for easier reading
            best_mask = mask.copy() ## copy the mask for later use
        mask_total = cv2.bitwise_or(mask_total, mask) ## combine the masks into one mask for display

    if best_mask is None or max_pixels < 8000: ## if no mask is found or the number of pixels is too low, send no cube to ESP32. eliminate false positives
        send_command("no cube")
        pending_detection = None
        time.sleep(0.2)
        cv2.imshow("Frame", frame) ## display the camera stream and mask for debugging
        cv2.imshow("Mask", mask_total) ## display the mask for debugging
        if cv2.waitKey(1) == 27: ## if ESC is pressed, break the loop and exit the program
            break
        continue

    kernel = np.ones((5, 5), np.uint8) ## create a kernel to clean up the mask
    mask = cv2.morphologyEx(best_mask, cv2.MORPH_CLOSE, kernel) ## close the mask to fill in holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) ## open the mask to remove noise

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) ## find the contours in the mask
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2) ## draw a rectangle around the ROI for visualization

    if contours: ## if contours are found, find the largest contour and draw it on the frame
        cnt = max(contours, key=cv2.contourArea) ## find the largest contour
        if cv2.contourArea(cnt) > 8000: ## if the area of the contour is too small, ignore it
            rect = cv2.minAreaRect(cnt) ## find the minimum area rectangle that encloses the contour
            box = cv2.boxPoints(rect) ## find the points of the rectangle
            box = np.int32(box + np.array([x1, y1])) ## add the x and y offsets to the points of the rectangle to get the actual points in the image
            cv2.drawContours(frame, [box], -1, (255, 0, 0), 2) ## draw the rectangle on the frame for visualization

            (cx, cy), _, _ = rect ## find the center of the rectangle
            center = (int(cx + x1), int(cy + y1)) ## add the x and y offsets to the center of the rectangle to get the actual center in the image
            cv2.circle(frame, center, 5, (0, 0, 255), -1) ## draw a circle at the center of the rectangle for visualization

            lengths = []
            for j in range(4):
                mid = ((box[j][0] + box[(j + 1) % 4][0]) // 2, (box[j][1] + box[(j + 1) % 4][1]) // 2) ## find the midpoints of the edges of the rectangle
                cv2.line(frame, mid, center, (0, 255, 0), 2) ## draw a line from the midpoint to the center of the rectangle for visualization
                lengths.append(find_distance(mid, center)) ## find the distance from the midpoint to the center of the rectangle for each edge

            if len(lengths) >= 2: ## calculate the average length of the midlines of the rectangle
                l1 = -0.00004735 * (lengths[0] ** 2) + 0.02291667 * lengths[0] - 0.06818182 #line of best fit for the calibration data. quadratic regression of the data accounts the perspective shift of the camera.
                l2 = -0.00004735 * (lengths[1] ** 2) + 0.02291667 * lengths[1] - 0.06818182
                avg = (l1 + l2) / 2 ## average the lengths of the midlines for better accuracy

                print(f"Detected size (real, pre-snap): {avg:.2f} inches") ## print the size of the cube in inches for debugging

                closest = min(CUBE_SIZES, key=lambda x: abs(x - avg)) ## find the closest size to the average length of the midlines of the rectangle. this is the best guess of the size 
                detection = f"{detected_color},{closest:.2f}" ## create a string with the color and size of the cube for sending to the ESP32. MUST FOLLOW THE FORMAT "COLOR,SIZE" FOR ESP32 TO WORK.

                if pending_detection == detection: ## if the detection is the same as the last detection, send the command to the ESP32, allows readings to stabilize before sending to ESP32
                    send_command(detection) 
                    pending_detection = None
                else:
                    print(f"Pending confirmation: {detection}") ## if the detection is different from the last detection, wait for confirmation from the ESP32 before sending the command.
                    pending_detection = detection
            else:
                send_command("no cube") ## if the lengths are not valid, send no cube to ESP32. eliminate false positives from the detection.
                pending_detection = None
        else:
            send_command("no cube") ## if the area of the contour is too small, send no cube to ESP32. eliminate false positives from the detection.
            pending_detection = None
    else:
        send_command("no cube") ## if no contours are found, send no cube to ESP32. eliminate false positives from the detection.
        pending_detection = None

    cv2.imshow("Frame", frame) ## display the camera stream for debugging
    cv2.imshow("Mask", mask) ## display the mask for debugging
    if cv2.waitKey(1) == 27: ## if ESC is pressed, break the loop and exit the program
        break

ser.close() ## if ESX pressed, close the serial port
cv2.destroyAllWindows() ## destroy all windows
