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

VELOCITY = 1
ALPHA = 3000
TOPDOWN = True
POSITION_NOT_SET = True
STEP = .25

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

p1Id = p.loadURDF("model/testcube.urdf", p1)
p2Id = p.loadURDF("model/testcube.urdf", p2)
p3Id = p.loadURDF("model/testcube.urdf", p3)


bzPoints = [p0, p1, p2, p3]

bezier = createBezier(bzPoints)


t = 0

#cam settings



cyaw=0
cpitch=-90
cdist=5

pos, orienation = p.getBasePositionAndOrientation(dogId)

if(TOPDOWN):
    cpitch = -89
p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

currentPoint = p1Id

while(1):
    keys = p.getKeyboardEvents()
    p.stepSimulation()

    #get point to change
    if keys.get(ord('1')):
        currentPoint = p1Id
    
    if keys.get(ord('2')):
        currentPoint = p2Id

    if keys.get(ord('3')):
        currentPoint = p3Id

    #change point position
    if keys.get(ord('h')) or keys.get(ord('b')) or keys.get(ord('n')) or keys.get(ord('m')):
        ptpos, _ = p.getBasePositionAndOrientation(currentPoint)

        if keys.get(ord('h')):
            newPosition = [ptpos[0], ptpos[1] + STEP, ptpos[2]]
        
        if keys.get(ord('b')):
            newPosition = [ptpos[0] - STEP, ptpos[1], ptpos[2]]

        if keys.get(ord('n')):
            newPosition = [ptpos[0], ptpos[1] - STEP, ptpos[2]]

        if keys.get(ord('m')):
            newPosition = [ptpos[0] + STEP, ptpos[1], ptpos[2]]

        p.resetBasePositionAndOrientation(currentPoint, newPosition, dogStartOrientation)

    # change camera zoom
    if keys.get(ord('z')):  #Z
        cdist+=.5
    if keys.get(ord('x')):  #X
        cdist-=.5

    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)


    if keys.get(ord(' ')): #D (change to space later)
        print('a')
        p.resetBasePositionAndOrientation(dogId, dogStartPos, dogStartOrientation)

        p1, _ = p.getBasePositionAndOrientation(p1Id)
        p2, _ = p.getBasePositionAndOrientation(p2Id)
        p3, _ = p.getBasePositionAndOrientation(p3Id)


        bzPoints = [p0, p1, p2, p3]

        bezier = createBezier(bzPoints)

        manhattan = 0
        for i in range(3):
            pt1 = bzPoints[i]
            pt2 = bzPoints[i+1]

            manhattan += sum(abs(np.subtract(pt1, pt2)))
            print(manhattan)

        ts = 1.0/(manhattan / VELOCITY)

        for t in np.arange(0, 1, ts):
            p.stepSimulation()
            targetPos = bezier(t)
            p.resetBasePositionAndOrientation(dogId, targetPos, dogStartOrientation)

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