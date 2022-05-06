import pybullet as pb
import pybullet_data as pbd
import time
import numpy as np

pysicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pbd.getDataPath())
pb.setGravity(0, 0, -10)

plane = pb.loadURDF('plane.urdf')

# 添加辅助线和文字
pb.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 2], lineColorRGB=[1, 0, 0])
pb.addUserDebugLine(lineFromXYZ=[1, 1, 0], lineToXYZ=[1, 1, 2], lineColorRGB=[0, 0, 1])
pb.addUserDebugText('LeftClick', textPosition=[0, 0, 2], textColorRGB=[1, 0, 0])
pb.addUserDebugText('RightClick', textPosition=[1, 1, 2], textColorRGB=[0, 0, 1])

pb.setRealTimeSimulation(enableRealTimeSimulation=1)

while True:
    events = pb.getMouseEvents()
    if events:
        for event in events:
            # 1鼠标移动，2鼠标点击
            if event[0] == 2:
                # 触发或松开了鼠标按键
                if event[4] == pb.KEY_WAS_TRIGGERED | pb.KEY_WAS_RELEASED:
                    # 0左键，1中键，2右键
                    if event[3] == 0:
                        ring_left = pb.loadURDF('model/rod_ring/ring.urdf', [0, 0, 2], pb.getQuaternionFromEuler([np.pi/2, 0, 0]))
                        pb.changeVisualShape(ring_left, -1, rgbaColor=[1, 0, 0, 1])
                    if event[3] == 2:
                        ring_right = pb.loadURDF('model/rod_ring/ring.urdf', [1, 1, 2], pb.getQuaternionFromEuler([np.pi/2, 0, 0]))
                        pb.changeVisualShape(ring_right, -1, rgbaColor=[0, 0, 1, 1])
    # 每隔一秒检查上一秒内的鼠标操作
    time.sleep(1)
