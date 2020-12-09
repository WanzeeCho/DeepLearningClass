import time
import pybullet as p
import pybullet_data
import numpy as np

#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
if (c < 0):
  c = p.connect(p.GUI)
# p.resetSimulation()

class HandSimulator:

    def __init__(self):


            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, 0)
            # print(c)
            # if (c < 0):
            #   p.connect(p.GUI)

            #load the MuJoCo MJCF hand
            objects_1 = p.loadMJCF("MPL/mpl2.xml")
            objects_2 = p.loadMJCF("MPL/mpl2.xml")

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
            p.setRealTimeSimulation(1)



    def step(self, actions):
        p.setJointMotorControl2(hand_1, 15, p.VELOCITY_CONTROL, index_1)


    def render(self):
        env_image = np.random.randn(300, 300) #TODO: actual imae
        return env_image



if __name__ == "__main__":
    #env = HandSimulator()

    #env_image = env.render()
    import os
    os.chdir(r'/Users/seohanseok/Dropbox/GitHub/bullet3/examples/pybullet/examples/')

    import __vrhand