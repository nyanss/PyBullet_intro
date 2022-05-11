import pybullet as pb
import pybullet_data
import time
from _create_multi_links_model import create_car

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -10)
pb.loadURDF('plane.urdf')

car = create_car()

pb.setRealTimeSimulation(enableRealTimeSimulation=1)

while pb.isConnected():
    events = pb.getMouseEvents()
    if events:
        for event in events:
            # 1鼠标移动，2鼠标点击
            if event[0] == 2:
                # 触发或松开了鼠标按键
                if event[4] == pb.KEY_WAS_TRIGGERED | pb.KEY_WAS_RELEASED:
                    # 0左键，1中键，2右键
                    if event[3] == 0:
                        for joint in range(pb.getNumJoints(car)):
                            pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=7, force=100)
                    if event[3] == 2:
                        for joint in range(pb.getNumJoints(car)):
                            pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=-7, force=100)
    # 每隔一秒检查上一秒内的鼠标操作
    time.sleep(1)
