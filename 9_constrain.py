import pybullet as pb
import time
import math

import pybullet_data
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

pb.loadURDF("plane.urdf")
cube = pb.loadURDF("cube.urdf", [1, 0, 1], globalScaling=0.4)
pb.changeVisualShape(cube, -1, rgbaColor=[1, 0, 0, 1])

constrain = pb.createConstraint(cube, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
pb.setRealTimeSimulation(enableRealTimeSimulation=1)

dt = 1.0 / 240.0
t = 0
while True:
    t += dt
    # 随时间转动的约束器
    pb.changeConstraint(constrain, [math.cos(3*t), math.sin(3*t), 1], maxForce=10)
    time.sleep(dt)
