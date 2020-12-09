#script to control a simulated robot hand using a VR glove
#see https://twitter.com/erwincoumans/status/821953216271106048
#and https://www.youtube.com/watch?v=I6s37aBXbV8
#vr glove was custom build using Spectra Symbolflex sensors (4.5")
#inside a Under Armour Batting Glove, using DFRobot Bluno BLE/Beetle
#with BLE Link to receive serial (for wireless bluetooth serial)

import serial
import time
import pybullet as p
import pybullet_data

#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
if (c < 0):
  c = p.connect(p.GUI)
p.resetSimulation()


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
# print(c)
# if (c < 0):
#   p.connect(p.GUI)

#load the MuJoCo MJCF hand
objects_1 = p.loadMJCF("/home/wan/MPL/mpl2.xml")
objects_2 = p.loadMJCF("/home/wan/MPL/mpl2.xml")

hand_1 = objects_1[0]
hand_2 = objects_2[0]



ho_1 = p.getQuaternionFromEuler([3.14/2,-3.14/2,-3.14/2])
ho_2 = p.getQuaternionFromEuler([3.14/2,-3.14/2,-3.14/2])
# hand_cid_1=p.resetBasePositionAndOrientation(hand_1, [-.5, 0, 0])
# hand_cid_2=p.resetBasePositionAndOrientation(hand_2, [-.5, 0, 0])
#
# p.setRealTimeSimulation(1)


hand_cid_1 = p.createConstraint(hand_1, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                              [0.500000, 0.0, 0.], ho_1)
print(p.getNumJoints(hand_1))
hand_cid_2 = p.createConstraint(hand_2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                              [-0.500000, 0.0, 0.], ho_2)



# 엄지
# p.setJointMotorControl2(hand_1, 5, p.POSITION_CONTROL, 1.3)
# p.setJointMotorControl2(hand_1, 7, p.POSITION_CONTROL, thumb_1)
# p.setJointMotorControl2(hand_1, 9, p.POSITION_CONTROL, thumb_1)
# p.setJointMotorControl2(hand_1, 11, p.POSITION_CONTROL, thumb_1)

# 검지
# p.setJointMotorControl2(hand_1, 15, p.POSITION_CONTROL, index_1)
# p.setJointMotorControl2(hand_1, 17, p.POSITION_CONTROL, index_1)
# p.setJointMotorControl2(hand_1, 19, p.POSITION_CONTROL, index_1)

# 중지
# p.setJointMotorControl2(hand_1, 22, p.POSITION_CONTROL, middle_1)
# p.setJointMotorControl2(hand_1, 24, p.POSITION_CONTROL, middle_1)
# p.setJointMotorControl2(hand_1, 26, p.POSITION_CONTROL, middle_1)

# 약지
# p.setJointMotorControl2(hand_1, 38, p.POSITION_CONTROL, pink_1)
# p.setJointMotorControl2(hand_1, 40, p.POSITION_CONTROL, pink_1)
# p.setJointMotorControl2(hand_1, 42, p.POSITION_CONTROL, pink_1)

# 새끼
# ringpos_1 = 0.5 * (pink_1 + middle_1)
# p.setJointMotorControl2(hand_1, 30, p.POSITION_CONTROL, ringpos_1)
# p.setJointMotorControl2(hand_1, 32, p.POSITION_CONTROL, ringpos_1)
# p.setJointMotorControl2(hand_1, 34, p.POSITION_CONTROL, ringpos_1)


# p.setJointMotorControl2(hand_1, 15, p.POSITION_CONTROL, index_1)

print("hand_cid")
print(hand_cid_1, hand_cid_2)

p.setRealTimeSimulation(1)


def step(actions):
    p.setJointMotorControl2(hand_1, 15, p.VELOCITY_CONTROL, index_1)


# step=0
# while True:
#     step+=1
#     print(step,end=' ')
#     if step<100:
#         pass
#     # for i in range(p.getNumJoints(hand_1)):
#     for index_1 in range(300):
#         for index_2 in range(300):
#             for index_3 in range(300):
#                 p.setJointMotorControl2(hand_1, 15, p.VELOCITY_CONTROL, index_1)
#                 p.setJointMotorControl2(hand_1, 17, p.POSITION_CONTROL, index_2)
#                 p.setJointMotorControl2(hand_1, 19, p.POSITION_CONTROL, index_3)
#
#             # print(p.setJointMotorControlArray(hand_1,  [0.,1], 0))
#     # for i in range(p.getNumJoints(hand_2)):
#         # print(p.setJointMotorControl2(hand_2, i, p.POSITION_CONTROL, 0, 0))
#
#
#     if step==400:
#         break
