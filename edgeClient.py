import cv2
import serial
import numpy as np
import time
import threading
from ultralytics import YOLO
import logging
from paho.mqtt import publish
import math
from datetime import datetime

# Variables
######################################################################
logging.getLogger("ultralytics").setLevel(logging.WARNING)
confidence_threshold = 0.4

# Bluetooth Configuration
TOF_PORT = "/dev/rfcomm1"
TOF_BAUD = 9600
BUTTON_PORT = '/dev/rfcomm0'
BUTTON_BAUD = 9600

# Video Configuration
VIDEO_PORT = '/dev/ttyACM0'
VIDEO_BAUD = 115200
WIDTH, HEIGHT = 160, 120
FRAME_SIZE = WIDTH * HEIGHT
FRAME_MARKER = b'\xAA\x00\xBB'
MARKER_LEN = len(FRAME_MARKER)
unitID = "001"

# Cloud Configuration
cloudAddress = 0 # insert address here, originally this had an AWS VM DNS name

# Multithreading configuration to mitigate issues from bottlenecks in video processing
tof_data = {"people": 0, "entries": 0, "exits": 0}
button_data = {"total_presses": 0, "last_time": None}
tof_lock = threading.Lock()
button_lock = threading.Lock()

# Press-event queue and counters
presses_queue = []  # {'time': ts, 'entries': 0}
lapsed_count = 0
yolo_detections = []

# Detection validation timeout (seconds)
DETECTION_WINDOW = 3
ENTRY_WINDOW = 15

entries_internal = 0
last_reported_entries = 0

# Initialize AI model
model = YOLO("yolo11n_ncnn_model", verbose=False)

# Initialize video serial
video_ser = serial.Serial(VIDEO_PORT, VIDEO_BAUD, timeout=1.0)
time.sleep(2)  # Allow Arduino reset

# Start Bluetooth threads
tof_thread = threading.Thread(target=tof_counter_thread, daemon=True)
tof_thread.start()

button_thread = threading.Thread(target=button_counter_thread, daemon=True)
button_thread.start()

# Video processing variables
buffer = bytearray()
frame_count = 0
last_byte_time = time.time()
last_frame_time = time.time()

################################################################
# Functions

# Print a warning to the console for debugging purposes and send data to AWS Cloud for SMS signalling
def alert(msg):
    logging.warning(msg)
    payload = f"Unit {unitID} has detected {msg}"
    try:
        publish.single(
            "personDetection",
            payload,
            cloudAddress
        )
    except Exception as e:
        print(f"Failed to publish MQTT alert: {e}")

# Validate that a person is seen within timeout
def validate_person(frame):
    global yolo_detections
    # Here begings the implementation of AI CV into the project
    results = model(frame, verbose=False)
    annotated_frame = results[0].plot()
    cv2.imshow("Tailgate detector", annotated_frame)
    for info in results:
        boxes = info.boxes
        for box in boxes:
            confidence = box.conf[0]
            Class = int(box.cls[0])
            if confidence > confidence_threshold:
                print("Person detected")
                # Add a record to a queue noting that a person has recetly been seen
                yolo_detections.append(time.time())
                # Remove detections older than the window
                yolo_detections = [t for t in yolo_detections if time.time() - t <= DETECTION_WINDOW]
                return True
     # Here ends the AI CV implementation
    return False

# Handle each entry event
def handle_entry_event(frame):
    global lapsed_count, yolo_detections
    now = time.time()
    date_time = datetime.now().replace(microsecond=0)

    # Expire old button presses
    while presses_queue and now - presses_queue[0]['time'] > ENTRY_WINDOW:
        press = presses_queue.pop(0)
        if press['entries'] == 0:
            lapsed_count += 1
            print("Lapsed button press (no entry within 15s)")

    # Check for person detections within 3 seconds of now
    detection_within_window = any(abs(now - det_time) <= DETECTION_WINDOW for det_time in yolo_detections)

    if detection_within_window:
        print(f"[TOF] Person detected within {DETECTION_WINDOW}s of entry")
    else:
        print(f"No person detected within {DETECTION_WINDOW}s of entry. ignoring as false positive")
        return

    # Handle entry event logic
    if presses_queue and now - presses_queue[0]['time'] <= ENTRY_WINDOW:
        press = presses_queue[0]
        press['entries'] += 1
        if press['entries'] == 1:
            print(f"Registered entry detected at {date_time}")
        if press['entries'] > 1:
            alert(f"Tailgating Event, Unregistered Entry at {date_time}")
    else:
        alert(f"Unregistered Entry detected at {date_time}")

# Processing data from the TOF IR tripwire system
def tof_counter_thread():
    bt_ser = None
    while True:
        try:
            if bt_ser is None or not bt_ser.is_open:
                bt_ser = serial.Serial(TOF_PORT, TOF_BAUD, timeout=2)
                print("[TOF] Connected successfully")

            if bt_ser.in_waiting:
                raw = bt_ser.readline()
                try:
                    # Parse the data from the TOF system
                    decoded = raw.decode('utf-8').strip()
                    parts = decoded.split(',')
                    print(parts) # Print the data from the motion detector for debugging purposes
                    if len(parts) == 3:
                        with tof_lock:
                            tof_data['people'] = int(parts[0]) # The net total of entries - exits. Not currently used, could be sent for reporting of traffic flow in a space
                            tof_data['entries'] = int(parts[1]) # Current implementation only uses this value
                            tof_data['exits'] = int(parts[2]) # Not currently used, as with the 'people' data
                                
                except Exception as e:
                    print(f"[TOF] Decode error: {e} (raw={raw})")
            time.sleep(0.05)
        
        except serial.SerialException as e:
            print(f"[TOF] Serial exception: {e}")
            if bt_ser:
                bt_ser.close()
            bt_ser = None
            time.sleep(2)
        except Exception as e:
            print(f"[TOF] Unexpected error: {e}")
            time.sleep(1)
        except:
            print("unknown TOF error")
            time.sleep(1)

# Processing data from the button registration system - A proxy for a NFC scanner. This same logic can be extended to such a system.
def button_counter_thread():
    """Thread to handle button presses"""
    button_ser = None
    
    while True:
        if button_ser is None:
            try:
                button_ser = serial.Serial(BUTTON_PORT, BUTTON_BAUD, timeout=2)
                print("[BUTTON] Connected successfully")
            except Exception as e:
                print(f"[BUTTON] Connection failed: {str(e)}")
                time.sleep(2)
                continue
        
        try:
            if button_ser.in_waiting:
                raw = button_ser.readline()
                try:
                    # Parse the data from the button registration system
                    decoded = raw.decode('utf-8').strip()
                    presses = int(decoded)
                    
                    # Print to console for debugging
                    print(f"Button pressed!")
                    
                    with button_lock:
                        button_data['total_presses'] = presses
                        button_data['last_time'] = time.time()
                    # Add an event to a FIFO queue                    
                    presses_queue.append({'time': button_data['last_time'], 'entries': 0})
                
                except ValueError:
                    print(f"[BUTTON] Received non-integer data: {raw}")
                except UnicodeDecodeError:
                    print(f"[BUTTON] Couldn't decode data: {raw}")
            else:
                time.sleep(0.1)
                
        except serial.SerialException:
            print("[BUTTON] Connection lost! Reconnecting...")
            button_ser.close()
            button_ser = None
        except Exception as e:
            print(f"[BUTTON] Error: {str(e)}")

########################################################################
# Begin main loop
running = True
try:
    while running:
        # Read video data
        chunk = video_ser.read(video_ser.in_waiting or 1)
        if chunk:
            buffer.extend(chunk)
            last_byte_time = time.time()
        
        # Process frame if marker found
        marker_pos = buffer.find(FRAME_MARKER)
        if marker_pos >= 0:
            frame_start = marker_pos + MARKER_LEN
            if len(buffer) - frame_start >= FRAME_SIZE:
                # Extract frame data
                frame_data = buffer[frame_start:frame_start + FRAME_SIZE]
                buffer = buffer[frame_start + FRAME_SIZE:]
                frame_count += 1
                
                try:
                    # Convert to OpenCV image
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = frame.reshape((HEIGHT, WIDTH))
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                    
                    # Person detection
                    validate_person(rgb_frame)
                    
                    # Get TOF person detection data
                    with tof_lock:
                        people = tof_data["people"]
                        entries = tof_data["entries"]
                        exits = tof_data["exits"]
                    
                    with tof_lock:
                        curr_reported_entries = int(tof_data['entries'])  # Current entries count reported by TOF

                        if curr_reported_entries != last_reported_entries:
                            # Calculate the number of new entries
                            new_entries = curr_reported_entries - last_reported_entries

                            # Update internal counter for valid positive changes
                            if new_entries > 0:
                                entries_internal += new_entries
                                last_reported_entries = curr_reported_entries
                                handle_entry_event(rgb_frame)
                            else:
                                # Handle unexpected resets or errors
                                last_reported_entries = curr_reported_entries  
                        
                except Exception as e:
                    print(f"[VIDEO] Processing error: {str(e)}")
                    
        # Handle buffer overflow
        elif len(buffer) > MARKER_LEN * 2:
            buffer = buffer[-MARKER_LEN:]
       
       # Check for video timeout
        if time.time() - last_byte_time > 2.0:
            print("[VIDEO] No data received for 2 seconds")
            time.sleep(0.1)
        
        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
finally:
    video_ser.close()
    cv2.destroyAllWindows()