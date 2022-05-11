import pybullet as pb
import pybullet_data
import time
import os

pysicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -10)

# 执行v-hacd凸分解算法
if not os.path.exists('model/rod_ring/_ring_vhacd.obj'):
    file_in = 'model/rod_ring/_ring.obj'
    file_out = 'model/rod_ring/_ring_vhacd.obj'
    log = 'model/vhacd_log.txt'
    pb.vhacd(file_in, file_out, log, resolution=100000000)

plane = pb.loadURDF('plane.urdf')

rod1 = pb.loadURDF('/model/rod_ring/rod.urdf', [1, -1, 0])
rod2 = pb.loadURDF('/model/rod_ring/rod.urdf', [1, 1, 0])

# 未分解的圆环（红色）
ring1 = pb.loadURDF('model/rod_ring/ring.urdf', [1, -1, 3], pb.getQuaternionFromEuler([1.57, 0, 0]))
pb.changeVisualShape(ring1, -1, rgbaColor=[1, 0, 0, 1])
pb.addUserDebugText('Concave model', [1, -1, 1], [1, 0, 0])

# 分解后的的圆环（蓝色）
ring2 = pb.loadURDF('model/rod_ring/ring_vhacd.urdf', [1, 1, 3], pb.getQuaternionFromEuler([1.57, 0, 0]))
pb.changeVisualShape(ring2, -1, rgbaColor=[0, 0, 1, 1])
pb.addUserDebugText('After Convex decomposition', [1, 1, 1], [0, 0, 1])

while pb.isConnected():
    pb.stepSimulation()
    time.sleep(1/24)
