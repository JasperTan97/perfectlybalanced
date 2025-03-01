import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version

p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
boxId = p.loadURDF("description/perfbalanced.urdf",cubeStartPos, cubeStartOrientation)
p.setGravity(0, 0, -9.81)
# for joint in range(p.getNumJoints(boxId)):
#    print(p.getJointState(boxId, joint))
#    p.setJointMotorControl2(boxId, joint, controlMode=p.TORQUE_CONTROL, force=0)

for i in range (10000):
   p.stepSimulation()
   time.sleep(1./240.)
   
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

p.disconnect()