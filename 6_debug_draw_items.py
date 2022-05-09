import pybullet as pb
import time
import pybullet_data

pb.connect(pb.GUI)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
kuka = pb.loadURDF("kuka_iiwa/model.urdf")
pb.addUserDebugText("tip", [0, 0, 0.1],
                    textColorRGB=[1, 0, 0],
                    textSize=1.5,
                    parentObjectUniqueId=kuka,
                    parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
pb.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=kuka, parentLinkIndex=6)
pb.setRealTimeSimulation(0)
angle = 0
while pb.isConnected():
    time.sleep(0.01)
    pb.resetJointState(kuka, 2, angle)
    pb.resetJointState(kuka, 3, angle)
    angle += 0.01
