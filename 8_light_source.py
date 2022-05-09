import pybullet as pb
import math
import time
import pybullet_data

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("r2d2.urdf", [0, 0, 1])
pb.loadURDF("plane.urdf")
pb.setGravity(0, 0, -10)

radius = 5
dt = 1.0 / 240.0
t = 0
pb.configureDebugVisualizer(shadowMapWorldSize=5)
pb.configureDebugVisualizer(shadowMapResolution=8192)

while True:
    t += dt
    pb.configureDebugVisualizer(lightPosition=[radius * math.sin(t), radius * math.cos(t), 3])

    pb.stepSimulation()
    time.sleep(dt)
