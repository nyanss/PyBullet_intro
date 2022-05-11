import pybullet as pb
import time
import pybullet_data
import math

pb.connect(pb.GUI)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
kuka = pb.loadURDF("kuka_iiwa/model.urdf")
pb.addUserDebugText("tip", [0, 0, 0.1],
                    textColorRGB=[1, 0, 0],
                    textSize=1.5,
                    parentObjectUniqueId=kuka,
                    parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0.5, 0, 0], [1, 0, 0], 3, parentObjectUniqueId=kuka, parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0, 0.5, 0], [0, 1, 0], 3, parentObjectUniqueId=kuka, parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0, 0, 0.5], [0, 0, 1], 3, parentObjectUniqueId=kuka, parentLinkIndex=6)
angle = 0
while pb.isConnected():
    pb.stepSimulation()
    pb.resetJointState(kuka, 2, angle)
    pb.resetJointState(kuka, 3, angle)
    link_state = pb.getLinkState(kuka, 6)
    pos = link_state[0]
    pb.resetDebugVisualizerCamera(cameraDistance=2,
                                  cameraYaw=0, cameraPitch=-30,
                                  cameraTargetPosition=pos)
    angle += 0.01
    time.sleep(1.0/240.0)
