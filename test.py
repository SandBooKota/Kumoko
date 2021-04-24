import pybullet as p
import time
import math
import numpy as np

from datetime import datetime

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
roboId = p.loadURDF("Prototype_URDF/kumo.urdf",cubeStartPos, cubeStartOrientation,
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
numJoints = 8

#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0, 0, 0, 0, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(numJoints):
  p.resetJointState(roboId, i, rp[i])

p.setGravity(0, 0, -10)

p.setRealTimeSimulation(1)

RL = 0.0
UpDown = 0.0

j1 = 0.0
j2 = 0.0

while 1:
  dt = datetime.now()
  time.sleep(1./240.)

  keys = p.getKeyboardEvents()
  for k,v in keys.items():
    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
        RL=0.01
    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
        RL=0
    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
        RL=-0.01
    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
        RL=0

    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
        UpDown=0.01
    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
        UpDown=0
    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
        UpDown=-0.01
    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
        UpDown=0

  j1 = j1 + RL
  j2 = j2 + UpDown

  jointPoses = [j1, j2, j1, j2, j1, j2, j1, j2]

  for i in range(numJoints):
      p.setJointMotorControl2(bodyIndex=roboId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=1,
                                force=10,
                                positionGain=0.1,
                                velocityGain=0.1)

#  jointPoses = [0, 0, 0, 0, 0, 0, 0, 0]
#
#  for i in range(numJoints):
#      p.setJointMotorControl2(bodyIndex=roboId,
#                              jointIndex=i,
#                              controlMode=p.POSITION_CONTROL,
#                              targetPosition=jointPoses[i],
#                              targetVelocity=10,
#                              force=100,
#                              positionGain=0.01,
#                              velocityGain=0.01)

  #jointInfo = p.getJointState(roboId, 1)
  #print(jointInfo[1])
