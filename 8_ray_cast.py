import pybullet as pb
import pybullet_data
from _create_multi_links_model import create_car
import time

pb.connect(pb.GUI)
pb.setGravity(0, 0, -10)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF('plane.urdf')

wall = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[1, 100, 2])
pb.createMultiBody(baseCollisionShapeIndex=wall, baseVisualShapeIndex=wall, basePosition=[20, 0, 1])
pb.createMultiBody(baseCollisionShapeIndex=wall, baseVisualShapeIndex=wall, basePosition=[-20, 0, 1])

car = create_car()
ray_length = 10
ray_color_hit = [1, 0, 0]
ray_color_free = [0, 1, 0]

pb.setRealTimeSimulation(1)
forward = 1

while pb.isConnected():
    pb.removeAllUserDebugItems()

    pos = pb.getBasePositionAndOrientation(car)[0]

    rayFrom_front = []
    rayTo_front = []
    rayFrom_behind = []
    rayTo_behind = []
    rays = []

    for i in range(5):
        rayFrom_front.append([pos[0] + 2, pos[1] - 0.8 + 0.4 * i, pos[2]])
        rayTo_front.append([rayFrom_front[-1][0] + ray_length, rayFrom_front[-1][1], rayFrom_front[-1][2]])
        rayFrom_behind.append([pos[0] - 2, pos[1] - 0.8 + 0.4 * i, pos[2]])
        rayTo_behind.append([rayFrom_behind[-1][0] - ray_length, rayFrom_behind[-1][1], rayFrom_behind[-1][2]])

    result_front = pb.rayTestBatch(rayFrom_front, rayTo_front)
    result_behind = pb.rayTestBatch(rayFrom_behind, rayTo_behind)

    for i in range(5):
        if result_front[i][0] in [-1, car]:
            pb.addUserDebugLine(rayFrom_front[i], rayTo_front[i], ray_color_free, 3)
        else:
            pb.addUserDebugLine(rayFrom_front[i], rayTo_front[i], ray_color_hit, 3)
            forward = -1
        if result_behind[i][0] in [-1, car]:
            pb.addUserDebugLine(rayFrom_behind[i], rayTo_behind[i], ray_color_free, 3)
        else:
            pb.addUserDebugLine(rayFrom_behind[i], rayTo_behind[i], ray_color_hit, 3)
            forward = 1

    for joint in range(pb.getNumJoints(car)):
        pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=-10*forward, force=100)

    time.sleep(1.0/240.0)
