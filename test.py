import numpy as np
import time
import math
from tqdm import tqdm
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client
from envs import env_builder
from robots import robot_config
from dataclasses import dataclass
from PIL import Image


from quadsim.robots import a1

def createBezier(pts):
    p0 = np.array(pts[0])
    p1 = np.array(pts[1])
    p2 = np.array(pts[2])
    p3 = np.array(pts[3])

    formula = lambda t : (pow(1-t, 3) * p0) + (3 * pow(1-t, 2) * t * p1) + (3 * (1-t) * pow(t, 2) * p2) + (pow(t, 3) * p3)

    return formula

def drawdebugSquares(pts):
    for pt in pts:
        pt = np.array(pt)
        r = 0.02
        #print(pt+[r,r,0] )
        p.addUserDebugLine(pt+[r,r,0]  ,pt+[r,-r,0] , [0,0,1])
        p.addUserDebugLine(pt+[r,-r,0] ,pt+[-r,-r,0], [0,0,1])
        p.addUserDebugLine(pt+[-r,-r,0],pt+[-r,r,0] , [0,0,1])
        p.addUserDebugLine(pt+[-r,r,0] ,pt+[r,r,0]  , [0,0,1] )
    return

def getStepPositions(pos,heading):
    pos = np.array([pos[0],pos[1]])
    rot = np.array([[np.cos(heading), -np.sin(heading)],
                   [np.sin(heading), np.cos(heading)]])

    forwardstep = 0.15
    backstep = 0.10
    sFL = np.matmul(rot,np.array([0.15 + forwardstep,0.15]))
    sFR = np.matmul(rot,np.array([0.15 + forwardstep,-0.15]))
    sRL = np.matmul(rot,np.array([-0.15 + backstep,0.15]))
    sRR = np.matmul(rot,np.array([-0.15 + backstep,-0.15]))
    sFL = np.concatenate((pos + sFL,np.zeros(1)))
    sFR = np.concatenate((pos + sFR,np.zeros(1)))
    sRL = np.concatenate((pos + sRL,np.zeros(1)))
    sRR = np.concatenate((pos + sRR,np.zeros(1)))
    FL = np.matmul(rot,np.array([0.15,0.15]))
    FR = np.matmul(rot,np.array([0.15,-0.15]))
    RL = np.matmul(rot,np.array([-0.15,0.15]))
    RR = np.matmul(rot,np.array([-0.15,-0.15]))
    FL = np.concatenate((pos + FL,np.zeros(1)))
    FR = np.concatenate((pos + FR,np.zeros(1)))
    RL = np.concatenate((pos + RL,np.zeros(1)))
    RR = np.concatenate((pos + RR,np.zeros(1)))

    return [FR, FL, RR, RL, sFR, sFL, sRR, sRL]

VELOCITY = 1/3000
ALPHA = 3000
TOPDOWN = True
POSITION_NOT_SET = True
STEP = .25


# Create PyBullet World
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -10)
planeId = p.loadURDF('plane.urdf')


# Create dog and grab ID
dogStartPos = [0, 0, 0.3]
dogStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
dogId = p.loadURDF("model/a1.urdf", [0, 0, 0.45], dogStartOrientation)
targetPos = dogStartPos
numJoints = p.getNumJoints(dogId)

# Joint Debug Info
#print("Number of Joints: " + str(numJoints))
#for i in range(numJoints):
#    print(p.getJointInfo(dogId, i))

#IMU = 0
    #h, hf, u, l, t
#FR: 1, 2, 3, 4, 5
#FL: 6, 7, 8, 9, 10
#RR: 11, 12, 13, 14, 15
#RL: 16, 17, 18, 19, 20
DEFAULT_JOINT_POS: np.ndarray = np.array([
-.1, .8, -1.5,
.1, .8, -1.5,
-.1, .8, -1.5,
.1, .8, -1.5
])
JOINTS = [
    1,3,4,
    6,8,9,
    11,13,14,
    16,18,19
]
p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=DEFAULT_JOINT_POS)


# Create Control Points
p0 = [0, 0, .3]
p1 = [2, 0, .3]
p2 = [3, 3, .3]
p3 = [0, 4, .3]

p1Id = p.loadURDF("model/testcube.urdf", p1)
p2Id = p.loadURDF("model/testcube.urdf", p2)
p3Id = p.loadURDF("model/testcube.urdf", p3)

# Create Stepping Stones
sFL = np.array([0.3,0.15,1e-4])
sFR = np.array([0.3,-0.15,1e-4])
sRL = np.array([-0.1,0.15,1e-4])
sRR = np.array([-0.1,-0.15,1e-4])
drawdebugSquares(getStepPositions(dogStartPos,0))


bzPoints = [p0, p1, p2, p3]

bezier = createBezier(bzPoints)

t = 0

# Camera Settings
cyaw=0
cpitch=-90
cdist=5

pos, orientation = p.getBasePositionAndOrientation(dogId)

if(TOPDOWN):
    cpitch = -89
p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

currentPoint = p1Id

closeup = True
if(closeup):
    p.resetDebugVisualizerCamera( cameraDistance=.75, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=dogStartPos)


# Execution Loop
while(1):
    keys = p.getKeyboardEvents()
    p.stepSimulation()
    #p.resetBasePositionAndOrientation(dogId, pos, dogStartOrientation)


    dogPos,_ = p.getBasePositionAndOrientation(dogId)

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
            p.resetDebugVisualizerCamera( cameraDistance=.75, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=dogPos)
        else:
            p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

    if not closeup:
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)
    
    if keys.get(ord('c')) and keys[ord('c')]&p.KEY_WAS_TRIGGERED:
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(dogId, [0,0,0.45], dogStartOrientation)
        p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=DEFAULT_JOINT_POS)
        sFL = np.array([0.3,0.15,1e-4])
        sFR = np.array([0.3,-0.15,1e-4])
        sRL = np.array([-0.1,0.15,1e-4])
        sRR = np.array([-0.1,-0.15,1e-4])
        drawdebugSquares([sFL, sFR, sRL, sRR])
        ([sFL, sFR, sRL, sRR])
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
    
    # TAKE A STEP
    if keys.get(ord('o')) and keys[ord('o')]&p.KEY_WAS_TRIGGERED:  
        FR, FL, RR, RL, sFR, sFL, sRR, sRL = getStepPositions(dogPos,0)
        stepAngles = p.calculateInverseKinematics2(dogId, [5,10,15,20], [sFR,FL,RR,sRL])
        p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=stepAngles)
    if keys.get(ord('p')) and keys[ord('p')]&p.KEY_WAS_TRIGGERED:  
        FR, FL, RR, RL, sFR, sFL, sRR, sRL = getStepPositions(dogPos,0)
        stepAngles = p.calculateInverseKinematics2(dogId, [5,10,15,20], [FR,sFL,sRR,RL])
        p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=stepAngles)
    #if keys.get(ord('i')) and keys[ord('i')]&p.KEY_WAS_TRIGGERED:  
    #    stepAngles = p.calculateInverseKinematics2(dogId, [1,6,11,16], [sFR+[0,0,.2],sFL+[0,0,.2],sRR+[0,0,.2],sRL+[0,0,.2]])
    #    p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=stepAngles)
    
    #dogPos, dogOrn = p.getBasePositionAndOrientation(dogId)
    #rigidBody.setWorldTransform
    #force = 300 * (np.array(targetPos) - np.array(dogPos))
    #p.applyExternalForce(objectUniqueId=dogId, linkIndex=-1,
    #                     forceObj=force, posObj=dogPos, flags=p.WORLD_FRAME)

    #--------------------------

    # RUNNING SIMULATION

    #--------------------------

    #if keys.get(ord(' ')): #D (change to space later)
    if ord(' ') in keys and keys[ord(' ')]&p.KEY_WAS_TRIGGERED:
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(dogId, dogStartPos, dogStartOrientation)
        drawdebugSquares([sFL, sFR, sRL, sRR])
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

        rightstep = True
        count = 0
        for t in np.arange(0, 1, ts):
            
            p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=targetPos)
            targetPos = bezier(t)
            heading = targetPos-lastPos
            #print(heading)
            yaw = math.atan2(targetPos[1]-lastPos[1], targetPos[0]-lastPos[0])
            #print(yaw)
            newOrientation = p.getQuaternionFromEuler([0,0,yaw])

            # Draw Path
            if (count % 500) == 0:

                # Draw Trail
                p.addUserDebugLine(lastPos,targetPos,[1,0,0]) 
                p.addUserDebugLine(lastPos,lastPos+heading,[0,0,1])
                lastPos = targetPos
            # Move Feet
            if (count % 1000) == 0:
                # Move leg
                FR, FL, RR, RL, sFR, sFL, sRR, sRL = getStepPositions(targetPos,yaw)
                drawdebugSquares([sFR, sFL, sRR, sRL])
                if rightstep:
                    stepAngles = p.calculateInverseKinematics2(dogId, [5,10,15,20], [sFR,FL,RR,sRL])
                else:
                    stepAngles = p.calculateInverseKinematics2(dogId, [5,10,15,20], [FR,sFL,sRR,RL])
                p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=stepAngles)
                rightstep = not rightstep

            #jointindices =    [ 1,   3,   4,   6,   8,   9,  11,  13,  14,  16,  18,  19]
            #targetpositions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            #forces =          [50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50]
            #p.setJointMotorControlArray(dogId,[8, 13],p.POSITION_CONTROL,targetPositions=[0.5, 0.5],forces=[50, 50])
            #p.setJointMotorControlArray(dogId,[3, 18],p.POSITION_CONTROL,targetPositions=[0.5, 0.5],forces=[1000, 1000])
            p.resetBasePositionAndOrientation(dogId, targetPos, newOrientation)
            p.stepSimulation()
            count += 1

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
