import mediapipe as mp
import cv2
import numpy as np

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)

def process_frames(frame, origin, locked, distance, tracking_vertical=False):
    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_tracking_vertical = tracking_vertical  # Persist mode
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    frame_copy = frame.copy()
    h, w, _ = frame.shape
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            palm_landmark = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            palm_x = int(palm_landmark.x * w)
            palm_y = int(palm_landmark.y * h)
            
            # FIST: all 4 fingers curled
            finger_tips = [8, 12, 16, 20]
            finger_pips = [6, 10, 14, 18]
            fist_detected = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y 
                              for tip, pip in zip(finger_tips, finger_pips))
            
            # INDEX ONLY UP (reset): index up + others curled
            index_up = hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y
            middle_curled = hand_landmarks.landmark[12].y > hand_landmarks.landmark[10].y
            ring_curled = hand_landmarks.landmark[16].y > hand_landmarks.landmark[14].y
            pinky_curled = hand_landmarks.landmark[20].y > hand_landmarks.landmark[18].y
            index_only_up = index_up and middle_curled and ring_curled and pinky_curled
            
            # INDEX + PINKY UP: index up, pinky up, middle+ring curled (thumb ignored)
            pinky_up = hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y
            middle_ring_curled = (hand_landmarks.landmark[12].y > hand_landmarks.landmark[10].y and
                                hand_landmarks.landmark[16].y > hand_landmarks.landmark[14].y)
            index_pinky_up = index_up and pinky_up and middle_ring_curled
            
            # Debug prints (remove later)
            print(f"Index up: {index_up}, Pinky up: {pinky_up}, Middle/ring curled: {middle_ring_curled}, Index+pinky: {index_pinky_up}")
            
            # Lock states (only when NOT already locked)
            if not origin_locked:
                if fist_detected:
                    origin_locked = True
                    origin_x = palm_x
                    origin_y = palm_y
                    current_tracking_vertical = False  # Horizontal mode
                    distance = 0
                    print("FIST LOCKED - HORIZONTAL MODE")
                elif index_pinky_up:
                    origin_locked = True
                    origin_x = palm_x
                    origin_y = palm_y
                    current_tracking_vertical = True  # Vertical mode
                    distance = 0
                    print("INDEX+PINKY LOCKED - VERTICAL MODE")
            
            # RESET with index only
            elif index_only_up:
                origin_locked = False
                origin_x = 0
                origin_y = 0
                distance = 0
                current_tracking_vertical = False
                print("INDEX ONLY - RESET")
            
            # Calculate distance if locked
            if origin_locked:
                if current_tracking_vertical:
                    distance = palm_y - origin_y
                else:
                    distance = palm_x - origin_x
            
            # Draw landmarks
            mp_drawing.draw_landmarks(frame_copy, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Fingertip feedback dots
            all_tips = [4, 8, 12, 16, 20]
            all_pips = [3, 6, 10, 14, 18]
            for tip_idx, pip_idx in zip(all_tips, all_pips):
                tip_pos = (int(hand_landmarks.landmark[tip_idx].x * w),
                          int(hand_landmarks.landmark[tip_idx].y * h))
                
                if tip_idx == 8:  # Index always green when up
                    color = (0, 255, 0) if hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y else (0, 0, 255)
                elif tip_idx == 20:  # Pinky green when up
                    color = (0, 255, 0) if hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y else (0, 0, 255)
                else:  # Others green when curled
                    color = (0, 255, 0) if hand_landmarks.landmark[tip_idx].y > hand_landmarks.landmark[pip_idx].y else (0, 0, 255)
                
                cv2.circle(frame_copy, tip_pos, 8, color, -1)
            
            # Tracking visualization
            if origin_locked:
                mode_text = "VERTICAL" if current_tracking_vertical else "HORIZONTAL"
                cv2.line(frame_copy, (origin_x, 0), (origin_x, h), (0, 255, 0), 3)
                cv2.line(frame_copy, (0, origin_y), (w, origin_y), (0, 255, 0), 3)
                cv2.circle(frame_copy, (palm_x, palm_y), 12, (0, 255, 0), -1)
                cv2.circle(frame_copy, (origin_x, origin_y), 10, (255, 255, 0), -1)
                
                cv2.putText(frame_copy, f'{mode_text}: {distance:.1f}px', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame_copy, 'INDEX UP = RESET', (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(frame_copy, 'FIST=Horizontal | INDEX+PINKY=Vertical', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                if index_pinky_up:
                    cv2.putText(frame_copy, 'INDEX+PINKY READY!', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    else:
        cv2.putText(frame_copy, 'No hand detected', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return frame_copy, (origin_x, origin_y), origin_locked, distance, current_tracking_vertical
