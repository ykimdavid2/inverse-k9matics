import pybullet as p
import pybullet_data
from dataclasses import dataclass
import numpy as np
import time
import math
from PIL import Image


from quadsim.robots import a1

def createBezier(pts):
    p0 = np.array(pts[0])
    p1 = np.array(pts[1])
    p2 = np.array(pts[2])
    p3 = np.array(pts[3])

    formula = lambda t : (pow(1-t, 3) * p0) + (3 * pow(1-t, 2) * t * p1) + (3 * (1-t) * pow(t, 2) * p2) + (pow(t, 3) * p3)

    return formula

def getBezierTangent(pts):
    p0 = np.array(pts[0])
    p1 = np.array(pts[1])
    p2 = np.array(pts[2])
    p3 = np.array(pts[3])

    formula = lambda t : 3*(pow(t,2)*(p3-p0+3*p1-3*p2)+2*t*(p0-2*p1+p3)-p0+p1)

    return formula

def moveleg():
    return

VELOCITY = 1/1500
ALPHA = 3000
TOPDOWN = True
POSITION_NOT_SET = True
STEP = .25

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -10)
planeId = p.loadURDF('plane.urdf')
dogStartPos = [0, 0, 0.5]
dogStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
dogId = p.loadURDF("model/a1.urdf", dogStartPos, dogStartOrientation)
targetPos = dogStartPos


#for i in range(21):
    #print(p.getJointInfo(dogId, i))
    
    #h, u, l
#FR: 1, 3, 4
#FL: 6, 8, 9
#RR: 11, 13, 14
#RL: 16, 18, 19

p0 = [0, 0, .5]
p1 = [2, 0, .5]
p2 = [3, 3, .5]
p3 = [0, 4, .5]

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

pos, orientation = p.getBasePositionAndOrientation(dogId)

if(TOPDOWN):
    cpitch = -89
p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

currentPoint = p1Id

closeup = False

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

        if keys.get(ord('h')) and keys[ord('h')]&p.KEY_WAS_TRIGGERED:
            newPosition = [ptpos[0], ptpos[1] + STEP, ptpos[2]]
        
        if keys.get(ord('b')) and keys[ord('b')]&p.KEY_WAS_TRIGGERED:
            newPosition = [ptpos[0] - STEP, ptpos[1], ptpos[2]]

        if keys.get(ord('n')) and keys[ord('n')]&p.KEY_WAS_TRIGGERED:
            newPosition = [ptpos[0], ptpos[1] - STEP, ptpos[2]]

        if keys.get(ord('m')) and keys[ord('m')]&p.KEY_WAS_TRIGGERED:
            newPosition = [ptpos[0] + STEP, ptpos[1], ptpos[2]]

        p.resetBasePositionAndOrientation(currentPoint, newPosition, dogStartOrientation)

    # change camera zoom
    if keys.get(ord('z')) and keys[ord('z')]&p.KEY_WAS_TRIGGERED:  #Z
        cdist+=.5
    if keys.get(ord('x')) and keys[ord('x')]&p.KEY_WAS_TRIGGERED:  #X
        cdist-=.5
    if keys.get(ord('q')) and keys[ord('q')]&p.KEY_WAS_TRIGGERED:
        closeup = not closeup

    if closeup:
        p.resetDebugVisualizerCamera( cameraDistance=.75, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=pos)
    else:
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)
    p.resetBasePositionAndOrientation(dogId, pos, dogStartOrientation)
    
    if keys.get(ord('c')) and keys[ord('c')]&p.KEY_WAS_TRIGGERED:
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(dogId, dogStartPos, dogStartOrientation)
        lastPos = dogStartPos
        targetPos = dogStartPos

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
        ts = 1.0/(manhattan / VELOCITY)

        linecount = 0
        for t in np.arange(0, 1, ts):
            if linecount == 3000:
                p.addUserDebugLine(lastPos,targetPos,[1,0,0])
                lastPos = targetPos
                linecount = 0
            targetPos = bezier(t)

            linecount += 1
        p.addUserDebugLine(lastPos,targetPos,[1,0,0])
        

    #--------------------------

    # RUNNING SIMULATION

    #--------------------------

    #if keys.get(ord(' ')): #D (change to space later)
    if ord(' ') in keys and keys[ord(' ')]&p.KEY_WAS_TRIGGERED:
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(dogId, dogStartPos, dogStartOrientation)
        lastPos = dogStartPos
        targetPos = dogStartPos

        p1, _ = p.getBasePositionAndOrientation(p1Id)
        p2, _ = p.getBasePositionAndOrientation(p2Id)
        p3, _ = p.getBasePositionAndOrientation(p3Id)


        bzPoints = [p0, p1, p2, p3]

        bezier = createBezier(bzPoints)
        tangent = getBezierTangent(bzPoints)

        manhattan = 0
        for i in range(3):
            pt1 = bzPoints[i]
            pt2 = bzPoints[i+1]

            manhattan += sum(abs(np.subtract(pt1, pt2)))
        ts = 1.0/(manhattan / VELOCITY)

        linecount = 0
        for t in np.arange(0, 1, ts):
            # Draw Path
            if linecount == 1000:
                p.addUserDebugLine(lastPos,targetPos,[1,0,0])
                lastPos = targetPos
                linecount = 0

            p.resetDebugVisualizerCamera( cameraDistance=.75, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=targetPos)
            p.stepSimulation()
            targetPos = bezier(t)
            heading = tangent(t)
            print(heading)
            yaw = math.atan2(heading[1], heading[0])
            print(yaw)
            newOrientation = p.getQuaternionFromEuler([0,0,yaw])

            #jointindices =    [ 1,   3,   4,   6,   8,   9,  11,  13,  14,  16,  18,  19]
            #targetpositions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            #forces =          [50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50]
            #p.setJointMotorControlArray(dogId,[8, 13],p.POSITION_CONTROL,targetPositions=[0.5, 0.5],forces=[50, 50])
            #p.setJointMotorControlArray(dogId,[3, 18],p.POSITION_CONTROL,targetPositions=[0.5, 0.5],forces=[1000, 1000])
            p.resetBasePositionAndOrientation(dogId, targetPos, newOrientation)
            linecount += 1

        p.addUserDebugLine(lastPos,targetPos,[1,0,0])
        for t in np.arange(0, 1, .005):
            p.stepSimulation()
    
    #dogPos, dogOrn = p.getBasePositionAndOrientation(dogId)
    #rigidBody.setWorldTransform
    #force = 300 * (np.array(targetPos) - np.array(dogPos))
    #p.applyExternalForce(objectUniqueId=dogId, linkIndex=-1,
    #                     forceObj=force, posObj=dogPos, flags=p.WORLD_FRAME)
    
    time.sleep(1./240.)
    #time.sleep(1./50.)

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
