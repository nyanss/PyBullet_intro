import pybullet as pb

pb.connect(pb.GUI)

# 加载模型并加入约束
body = pb.loadURDF('model/multi_body.urdf')
constraint = pb.createConstraint(parentBodyUniqueId=body, parentLinkIndex=2,
                                 childBodyUniqueId=body, childLinkIndex=-1,
                                 jointType=pb.JOINT_POINT2POINT, jointAxis=[0, 0, 0],
                                 parentFramePosition=[2, 0, -0.2], childFramePosition=[-1, 0, 0.2])
pb.changeConstraint(constraint, maxForce=1000)

# 不加约束的模型
body_without_constraint = pb.loadURDF('model/multi_body.urdf', [0, 0, 1])
for i in range(4):
    pb.changeVisualShape(body_without_constraint, i-1, rgbaColor=[1, 0, 0, 1])

pb.setRealTimeSimulation(1)
