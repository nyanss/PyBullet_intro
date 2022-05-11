import pybullet as pb
import math


def create_car():
    # 车的主体部分作为base_link
    box_mass = 1
    box = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[2, 1, 0.2])
    box_position = [0, 0, 2]

    # 四个车轮的相关参数
    wheels_masses = [1 for _ in range(4)]
    wheels = [pb.createCollisionShape(pb.GEOM_CYLINDER, radius=0.5, height=0.4) for _ in range(4)]
    wheels_visual = [-1 for _ in range(4)]
    wheel_positions = [[2, 1.2, 0], [2, -1.2, 0], [-2, 1.2, 0], [-2, -1.2, 0]]
    wheel_orns = [pb.getQuaternionFromEuler([math.pi/2, 0, 0]) for _ in range(4)]
    wheel_inertial_positions = [[0, 0, 0] for _ in range(4)]
    wheel_inertial_orns = [[0, 0, 0, 1] for _ in range(4)]

    # 车轮作为child link需要指定parent link
    parent_links = [0 for _ in range(4)]

    # link之间的joint类型与取向
    joint_types = [pb.JOINT_REVOLUTE for _ in range(4)]
    joint_axis = [[0, 0, 1] for _ in range(4)]

    # 组装成一个模型
    # link相关的参数都不能省略不然会报错
    car = pb.createMultiBody(baseMass=box_mass,
                             baseCollisionShapeIndex=box,
                             baseVisualShapeIndex=-1,
                             basePosition=box_position,
                             linkMasses=wheels_masses,
                             linkCollisionShapeIndices=wheels,
                             linkVisualShapeIndices=wheels_visual,
                             linkPositions=wheel_positions,
                             linkOrientations=wheel_orns,
                             linkInertialFramePositions=wheel_inertial_positions,
                             linkInertialFrameOrientations=wheel_inertial_orns,
                             linkParentIndices=parent_links,
                             linkJointTypes=joint_types,
                             linkJointAxis=joint_axis)
    return car
