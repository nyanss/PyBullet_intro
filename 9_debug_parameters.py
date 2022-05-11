import pybullet as pb
import pybullet_data
import time
from _create_multi_links_model import create_car

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -10)
pb.loadURDF('plane.urdf')

car = create_car()

motor_controller = pb.addUserDebugParameter('MotorVelocity', -20, 20, 0)
gx_slider = pb.addUserDebugParameter('Gravity_x', -10, 10, 0)
gy_slider = pb.addUserDebugParameter('Gravity_y', -10, 10, 0)
gz_slider = pb.addUserDebugParameter('Gravity_z', -10, 10, -10)

pb.setRealTimeSimulation(1)

while pb.isConnected():
    motor_velocity = pb.readUserDebugParameter(motor_controller)
    gx = pb.readUserDebugParameter(gx_slider)
    gy = pb.readUserDebugParameter(gy_slider)
    gz = pb.readUserDebugParameter(gz_slider)
    pb.setGravity(gx, gy, gz)
    for joint in range(pb.getNumJoints(car)):
        pb.setJointMotorControl2(car, joint, pb.VELOCITY_CONTROL, targetVelocity=motor_velocity, force=100)

    time.sleep(1.0/240.0)
