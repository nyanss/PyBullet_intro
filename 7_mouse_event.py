import pybullet as pb
import pybullet_data
import time
import math

# 到开始模拟前，代码同3_multi_links_model.py
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -10)
plane = pb.createCollisionShape(pb.GEOM_PLANE)
pb.createMultiBody(baseCollisionShapeIndex=plane, baseVisualShapeIndex=-1)
box_mass = 1
box = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[2, 1, 0.2])
box_position = [0, 0, 2]
wheels_masses = [1 for _ in range(4)]
wheels = [pb.createCollisionShape(pb.GEOM_CYLINDER, radius=0.5, height=0.4) for _ in range(4)]
wheels_visual = [-1 for _ in range(4)]
wheel_positions = [[2, 1.2, 0], [2, -1.2, 0], [-2, 1.2, 0], [-2, -1.2, 0]]
whell_orns = [pb.getQuaternionFromEuler([math.pi/2, 0, 0]) for _ in range(4)]
wheel_inertial_positions = [[0, 0, 0] for _ in range(4)]
wheel_inertial_orns = [[0, 0, 0, 1] for _ in range(4)]
parent_links = [0 for _ in range(4)]
joint_types = [pb.JOINT_REVOLUTE for _ in range(4)]
joint_axis = [[0, 0, 1] for _ in range(4)]
car = pb.createMultiBody(baseMass=box_mass,
                         baseCollisionShapeIndex=box,
                         baseVisualShapeIndex=-1,
                         basePosition=box_position,
                         linkMasses=wheels_masses,
                         linkCollisionShapeIndices=wheels,
                         linkVisualShapeIndices=wheels_visual,
                         linkPositions=wheel_positions,
                         linkOrientations=whell_orns,
                         linkInertialFramePositions=wheel_inertial_positions,
                         linkInertialFrameOrientations=wheel_inertial_orns,
                         linkParentIndices=parent_links,
                         linkJointTypes=joint_types,
                         linkJointAxis=joint_axis)
for joint in range(pb.getNumJoints(car)):
    pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=7, force=100)

pb.setRealTimeSimulation(enableRealTimeSimulation=1)

# mouse event部分
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
