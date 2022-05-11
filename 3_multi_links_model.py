import pybullet as pb
import pybullet_data
from _create_multi_links_model import create_car

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -10)

plane = pb.createCollisionShape(pb.GEOM_PLANE)
# -1表示碰撞模型与视觉模型一致
pb.createMultiBody(baseCollisionShapeIndex=plane, baseVisualShapeIndex=-1)

car = create_car()
# 在joint处设置匀速电机
for joint in range(pb.getNumJoints(car)):
    pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=7, force=100)

pb.setRealTimeSimulation(enableRealTimeSimulation=1)
