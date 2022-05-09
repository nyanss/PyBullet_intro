import pybullet as pb
import pybullet_data as pbd

pysicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pbd.getDataPath())
pb.setGravity(0, 0, -10)

plane = pb.loadURDF('plane.urdf')

rod1 = pb.loadURDF('/model/rod_ring/rod.urdf', [1, -1, 0])
rod2 = pb.loadURDF('/model/rod_ring/rod.urdf', [1, 1, 0])

# 未分解的圆环（红色）
ring1 = pb.loadURDF('model/rod_ring/ring.urdf', [1, -1, 3], pb.getQuaternionFromEuler([1.57, 0, 0]))
pb.changeVisualShape(ring1, -1, rgbaColor=[1, 0, 0, 1])

# 分解成48份的的圆环（蓝色）
ring2 = pb.loadURDF('model/rod_ring/ring_48.urdf', [1, 1, 3])
[pb.changeVisualShape(ring2, i - 1, rgbaColor=[0, 0, 1, 1]) for i in range(48)]

pb.setRealTimeSimulation(enableRealTimeSimulation=1)
