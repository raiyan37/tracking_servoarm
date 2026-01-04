import time
import cv2
import mediapipe as mp
import numpy as np
import serial

# Initialize MediaPipe Hand solution
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Initialize serial connection to Arduino
# Replace COM3 w/ serial path
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected successfully!")
except Exception as e:
    print(f"Error: Could not connect to Arduino: {e}")
    print("Running in demo mode (no Arduino connection)")
    arduino = None

# Initialize webcam
cap = cv2.VideoCapture(0)

# Pre-define constants to avoid repeated allocations
FINGER_NAMES = ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky')
FINGER_POINTS = [
    [1, 2, 3, 4],      # thumb
    [5, 6, 7, 8],      # index
    [9, 10, 11, 12],   # middle
    [13, 14, 15, 16],  # ring
    [17, 18, 19, 20]   # pinky
]

def calculate_distance_sq(point1, point2):
    """Calculate squared Euclidean distance (faster, no sqrt)"""
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    dz = point1.z - point2.z
    return dx*dx + dy*dy + dz*dz

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return np.sqrt(calculate_distance_sq(point1, point2))

def get_finger_curl(landmarks, finger_points):
    """
    Calculate finger curl based on distances between joints
    Returns 0-180 where 0 is open and 180 is closed
    """
    # Calculate total path length through joints
    total_length = sum(
        calculate_distance(landmarks[finger_points[i]], landmarks[finger_points[i + 1]])
        for i in range(len(finger_points) - 1)
    )
    
    # Direct distance from base to tip
    straight_length = calculate_distance(landmarks[finger_points[0]], landmarks[finger_points[-1]])
    
    # Calculate curl ratio
    curl_ratio = max(0.0, min(1.0, (1 - straight_length / total_length) * 2)) if total_length > 0 else 0.0
    
    return int(curl_ratio * 180)

def get_thumb_curl(landmarks):
    """Special calculation for thumb"""
    # Distance from thumb tip to index finger base
    tip_to_index = calculate_distance(landmarks[4], landmarks[5])
    
    # Distance from thumb base to tip
    base_to_tip = calculate_distance(landmarks[1], landmarks[4])
    
    # Calculate curl ratio
    curl_ratio = max(0.0, min(1.0, (1 - tip_to_index / (base_to_tip + 0.01)) * 1.5))
    
    return int(curl_ratio * 180)

print("Starting hand tracking... Press 'q' to quit.")
print("Show your hand to the camera and move your fingers!")

frame_count = 0

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Failed to capture image")
        continue

    frame_count += 1
    
    # Flip image horizontally for mirror view
    image = cv2.flip(image, 1)
    
    # MediaPipe works fine with BGR, no need to convert
    image.flags.writeable = False  # Improve performance
    results = hands.process(image)
    image.flags.writeable = True
    
    # Check if hand detected (only one hand since max_num_hands=1)
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]  # Get first (and only) hand
        
        # Draw landmarks
        mp_drawing.draw_landmarks(
            image, 
            hand_landmarks, 
            mp_hands.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
        )
        
        landmarks = hand_landmarks.landmark
        
        # Calculate all angles efficiently
        angles = [
            get_thumb_curl(landmarks),
            *(get_finger_curl(landmarks, points) for points in FINGER_POINTS[1:])
        ]
        
        # Send data to Arduino (every 3 frames)
        if arduino and frame_count % 3 == 0:
            command = f"{','.join(map(str, angles))}\n"
            try:
                arduino.write(command.encode())
            except Exception as e:
                print(f"Error sending to Arduino: {e}")
        
        # Display angles with color coding
        for i, (name, angle) in enumerate(zip(FINGER_NAMES, angles)):
            # Color interpolation: green (open) to red (closed)
            color = (0, 255 - (angle * 255 // 180), angle * 255 // 180)
            cv2.putText(image, f"{name}: {angle:3d}", (10, 30 + i * 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Status text
        status_text = "Arduino: Connected" if arduino else "Arduino: Disconnected"
        cv2.putText(image, status_text, (10, image.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    else:
        # No hand detected
        cv2.putText(image, "No hand detected - Show your hand!", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Display the image
    cv2.imshow('MediaPipe Hand Tracking - Press Q to Quit', image)
    
    # Exit on 'q' key
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
hands.close()
if arduino:
    arduino.close()
print("Program ended.")