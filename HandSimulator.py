import time
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

# first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.GUI)
#c = p.connect(p.DIRECT)


# p.resetSimulation()
class HandSimulator:



    def convertSensor(x, fingerIndex):

        minVarray = [275, 280, 350, 290]
        maxVarray = [450, 550, 500, 400]

        minV = minVarray[fingerIndex]
        maxV = maxVarray[fingerIndex]

        v = minV
        try:
            v = float(x)
        except ValueError:
            v = minV
        if (v < minV):
            v = minV
        if (v > maxV):
            v = maxV
        b = (v - minV) / float(maxV - minV)
        return (b)

    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)
        # print(c)
        # if (c < 0):
        #   p.connect(p.GUI)

        # load the MuJoCo MJCF hand
        objects_1 = p.loadMJCF("MPL/mpl2.xml")
        #objects_2 = p.loadMJCF("MPL/mpl2.xml")

        hand_1 = objects_1[0]
        #hand_2 = objects_2[0]

        ho_1 = p.getQuaternionFromEuler([3.14 / 2, -3.14 / 2, -3.14 / 2])
        #ho_2 = p.getQuaternionFromEuler([3.14 / 2, -3.14 / 2, -3.14 / 2])
        # hand_cid_1=p.resetBasePositionAndOrientation(hand_1, [-.5, 0, 0])
        # hand_cid_2=p.resetBasePositionAndOrientation(hand_2, [-.5, 0, 0])
        #
        # p.setRealTimeSimulation(1)

        # p.createConstraint(hand_1, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
        #                                 [0.500000, 0.0, 0.], ho_1)
        # print(p.getNumJoints(hand_1))
        # hand_cid_2 = p.createConstraint(hand_2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
        #                                 [-0.500000, 0.0, 0.], ho_2)
        # p.setRealTimeSimulation(1)
        # p.set

        self.local_mapping = {
                                "THUMB_CMC": 5, "THUMB_MCP":7, "THUMB_IP":9, "THUMB_TIP":11,
                                "INDEX_FINGER_MCP": 13, "INDEX_FINGER_PIP": 15, "INDEX_FINGER_DIP": 17,
                                "MIDDLE_FINGER_MCP": 22, "MIDDLE_FINGER_PIP": 24, "MIDDLE_FINGER_DIP": 26,
                                "RING_FINGER_MCP": 30, "RING_FINGER_PIP": 32, "RING_FINGER_DIP": 34,
                                "PINKY_MCP": 38, "PINKY_PIP": 40, "PINKY_DIP": 42
                              }
        self.hand_1 = hand_1
        self.pos_update_vec_prev = None
        self.step_counter = 0
        # for i in range(1000):
        #     p.setJointMotorControl2(self.hand_1, 15, p.POSITION_CONTROL, i)

    def step(self, pos_update_vec):
        p.setRealTimeSimulation(1)
        print(pos_update_vec)
        # calc velocity from prev
        vel_update_vecs = {}
        if self.pos_update_vec_prev is not None:
            for joint_name, xyz_value in pos_update_vec.items():
                xyz_vel = xyz_value - self.pos_update_vec_prev[joint_name]
                vel_update_vecs[joint_name] = xyz_vel
        # -------- hand simulator code ----------x
            # {"pinky1": [0.3, -0.2, 0.9] # velocity x y z
        for joint_name, velocity_update_vec in vel_update_vecs.items():
            if joint_name in self.local_mapping:
                local_joint_idx = self.local_mapping[joint_name]
                p.setJointMotorControl2(self.hand_1, local_joint_idx, p.POSITION_CONTROL, velocity_update_vec[0]*10)
                print(velocity_update_vec[0]*10)
                # print("Applying vel")
            else:
                print("Missing joint", joint_name)
        # =- -------------------------------------

        self.step_counter += 1
        self.pos_update_vec_prev = pos_update_vec

    def render(self):
        camTargetPos = [0, 0, 0]
        cameraUp = [0, 1, 0]
        cameraPos = [0, 1, 0]
        yaw = -90 #+ self.step_counter % 360
        pitch = 90
        roll = -180
        upAxisIndex = 2
        camDistance = 0.5
        pixelWidth = 640
        pixelHeight = 480
        nearPlane = 0.01
        farPlane = 100
        fov = 60
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch,
                                                         roll, upAxisIndex)
        aspect = pixelWidth / pixelHeight
        projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)

        img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix, projectionMatrix)
        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgb = img_arr[2]  # color data RGB
        dep = img_arr[3]  # depth data
        print("w=", w, "h=", h)
        np_img_arr = np.reshape(rgb, (h, w, 4))
        np_img_arr = np_img_arr * (1. / 255.)

        env_image = np_img_arr
        #plt.imshow(env_image)
        #plt.show()
        # env_image = np.random.randn(300, 300) #TODO: actual imae
        return env_image


if __name__ == "__main__":
    env = HandSimulator()
    env_image = env.render()
