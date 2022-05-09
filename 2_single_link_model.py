import pybullet as pb
import pybullet_data as pbd

pysicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pbd.getDataPath())
pb.setGravity(0, 0, -10)

plane = pb.loadURDF('plane.urdf')

# 可以自由运动的模型(绿色)
cylinder = pb.loadURDF('model/simple_shape.urdf', [0, 0, 1])
pb.changeVisualShape(cylinder, -1, rgbaColor=[0, 1, 0, 1])

# 加载时强行固定模型（红色）
cylinder_static = pb.loadURDF('model/simple_shape.urdf', [1, 1, 1], useFixedBase=True)
pb.changeVisualShape(cylinder_static, -1, rgbaColor=[1, 0, 0, 1])

# urdf里设置了0转动惯量，但第一个模型可以转动，因为软件自动赋予模型转动惯量
# 通过flags参数调用urdf文件里的转动惯量，得到不能转动的模型（蓝色）
cylinder_no_rotation = pb.loadURDF('model/simple_shape.urdf', [1, -1, 1], flags=pb.URDF_USE_INERTIA_FROM_FILE)
pb.changeVisualShape(cylinder_no_rotation, -1, rgbaColor=[0, 0, 1, 1])

pb.setRealTimeSimulation(enableRealTimeSimulation=1)
