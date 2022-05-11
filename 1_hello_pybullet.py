import pybullet as pb
import pybullet_data
import time

# 创建仿真环境，并打开图形界面
pysicsClient = pb.connect(pb.GUI)

# 设置额外搜索路径，加载模型文件时会在工作路径和此路径下搜索
# 该路径默认为：%python安装路径%/Lib/site-packages/pybullet_data/
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置仿真环境的重力
pb.setGravity(0, 0, -10)

# 加载模型
plane = pb.loadURDF('plane.urdf')

# 设置模型的初始位置和姿态，并加载模型
cubePos = [0, 0, 10]
# 软件里默认的方位角表示方式不是欧拉角，需要用getQuaternionFromEuler()转换
cubeOrt = pb.getQuaternionFromEuler([0, 0, 0])
cube = pb.loadURDF('cube.urdf', cubePos, cubeOrt)

# 开始实时仿真
pb.setRealTimeSimulation(enableRealTimeSimulation=1)

# 开始步进仿真
'''while pb.isConnected():
    pb.stepSimulation()
    time.sleep(1.0/240)'''

