import numpy as np 
# simulator

# class HandSimulator():
#
#     def __init__(self):
#         self.pos_update_vec_prev=None
#
#     def step(self, pos_update_vec):
#         print(pos_update_vec)
#         # calc velocity from prev
#         vel_update_vecs = {}
#         if self.pos_update_vec_prev is not None:
#             for joint_name, xyz_value in pos_update_vec.items():
#                 xyz_vel = xyz_value - self.pos_update_vec_prev[joint_name]
#                 vel_update_vecs[joint_name] = xyz_vel
#         # -------- hand simulator code ----------
#
#
#         # ----------------------------------------
#         self.pos_update_vec_prev = pos_update_vec
#         return
#
#     def render(self):
#         return np.random.randn(300, 300)
        
import cv2
import mediapipe as mp

from HandSimulator import HandSimulator

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# For webcam input:
hands = mp_hands.Hands(
    min_detection_confidence=0.7, min_tracking_confidence=0.5)
env = HandSimulator()

cap = cv2.VideoCapture(0)
while cap.isOpened():
  success, image = cap.read()
  if not success:
    break

  # Flip the image horizontally for a later selfie-view display, and convert
  # the BGR image to RGB.https://github.com/WanzeeCho/DeepLearningClass_assignment
  image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
  # To improve performance, optionally mark the image as not writeable to
  # pass by reference.
  image.flags.writeable = False
  results = hands.process(image)

  # Draw the hand annotations on the image.
  image.flags.writeable = True
  image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
  if results.multi_hand_landmarks:
    for hand_landmarks in results.multi_hand_landmarks:
      #print(hand_landmarks)
      #print(mp_hands.HAND_CONNECTIONS)
      mp_drawing.draw_landmarks(
          image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
      # convert hand tracking to coordinates
      pos_update_vec = {}
      for idx, joint_names_tuple in enumerate(mp_hands.HAND_CONNECTIONS):
          pos_vec = hand_landmarks.landmark[idx]
          xyz = [pos_vec.x, pos_vec.y, pos_vec.z]
          joint_name = str(joint_names_tuple[1]).split(".")[-1]
          pos_update_vec[joint_name]= np.array(xyz)
      env.step(pos_update_vec)
    #break
  cv2.imshow('MediaPipe Hands', image)
  env_image = env.render()
  cv2.imshow('Simulator', env_image)
  if cv2.waitKey(5) & 0xFF == 27:
    break
hands.close()
cap.release()
