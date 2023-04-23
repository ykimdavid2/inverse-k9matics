import pybullet as p
import pybullet_data
from dataclasses import dataclass
import numpy as np
import time
from PIL import Image



def createBezier(pts):
    p0 = np.array(pts[0])
    p1 = np.array(pts[1])
    p2 = np.array(pts[2])
    p3 = np.array(pts[3])

    formula = lambda t : (pow(1-t, 3) * p0) + (3 * pow(1-t, 2) * t * p1) + (3 * (1-t) * pow(t, 2) * p2) + (pow(t, 3) * p3)

    return formula

DURATION = 250
ALPHA = 3000
TOPDOWN = True
POSITION_NOT_SET = True

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -10)
planeId = p.loadURDF('plane.urdf')
dogStartPos = [0, 0, 0.5]
dogStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
dogId = p.loadURDF("model/a1.urdf", dogStartPos, dogStartOrientation)
targetPos = [2, 2, 1]

p0 = [0, 0, .5]
p1 = [2, 0, .5]
p2 = [0, 4, .5]
p3 = [3, 3, .5]

p.loadURDF("r2d2.urdf", p1)
p.loadURDF("r2d2.urdf", p2)
p.loadURDF("r2d2.urdf", p3)


bzPoints = [p0, p1, p2, p3]

bezier = createBezier(bzPoints)

ts = 1 / DURATION

t = 0

#cam settings



cyaw=0
cpitch=-90
cdist=5

pos, orienation = p.getBasePositionAndOrientation(dogId)

if(TOPDOWN):
    cpitch = -89
p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

for t in np.arange(0, 1, ts):
    print(t)
    p.stepSimulation()
    targetPos = bezier(t)
    p.resetBasePositionAndOrientation(dogId, targetPos, dogStartOrientation)
    print(targetPos)

    #dogPos, dogOrn = p.getBasePositionAndOrientation(dogId)
    #rigidBody.setWorldTransform
    #force = 300 * (np.array(targetPos) - np.array(dogPos))
    #p.applyExternalForce(objectUniqueId=dogId, linkIndex=-1,
    #                     forceObj=force, posObj=dogPos, flags=p.WORLD_FRAME)
    
    time.sleep(1./240.)

"""
while(1):
    p.stepSimulation()


    dogPos, dogOrn = p.getBasePositionAndOrientation(dogId)

    force = 300 * (np.array(targetPos) - np.array(dogPos))
    p.applyExternalForce(objectUniqueId=dogId, linkIndex=-1,
                         forceObj=force, posObj=dogPos, flags=p.WORLD_FRAME)

    #keys = p.getKeyboardEvents()
    time.sleep(1./240.)

    #do nothing
    """

# Run the simulation for a fixed amount of steps.
"""for i in range(20):
    position, orientation = p.getBasePositionAndOrientation(r2d2)
    x, y, z = position
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
    print(f"{i:3}: x={x:0.10f}, y={y:0.10f}, z={z:0.10f}), roll={roll:0.10f}, pitch={pitch:0.10f}, yaw={yaw:0.10f}")
    p.stepSimulation()


for i in range (10000):
   p.stepSimulation()
   time.sleep(1./100.)"""

#p.disconnect()

"""
def getPositionOnClick():
    if (POSITION_NOT_SET):
        mouseEvents = p.getMouseEvents(physicsClient)
        if (len(mouseEvents) > 0 and mouseEvents[0][0] == 2):
            mouseEvent = mouseEvents[0]
            #POSITION_NOT_SET = False
            mouseX = mouseEvent[1]
            mouseY = mouseEvent[2]

            cam = p.getDebugVisualizerCamera()
            camX = cam[0]
            camY = cam[1]

            vec = np.zeros(4)

            viewM = np.array(cam[2]).reshape(4,4)
            projM = np.array(cam[3]).reshape(4,4)

            M = viewM @ projM
            M_inv = np.linalg.inv(M)

            vec[0] = (2.0 * (mouseX / camX)) - 1.0
            vec[1] = 1.0 - (2.0 * (mouseY / camY))
            vec[2] = 1.0
            vec[3] = 1.0

            pos = vec @ M_inv

            w = pos[3]
            pos[0] /= w
            pos[1] /= w
            pos[2] = 0


            print(pos)
            targetPos = [pos[0], pos[1], 0]
"""