import numpy as np
import time
import math
# from tqdm import tqdm
import pybullet as p
import pybullet_data
# from pybullet_utils import bullet_client
# from envs import env_builder
from robots import robot_config
# from dataclasses import dataclass
# from PIL import Image
from util import leg_ik, foot_path


# from quadsim.robots import a1

def createBezier(pts):
    p0 = np.array(pts[0])
    p1 = np.array(pts[1])
    p2 = np.array(pts[2])
    p3 = np.array(pts[3])

    formula = lambda t : (pow(1-t, 3) * p0) + (3 * pow(1-t, 2) * t * p1) + (3 * (1-t) * pow(t, 2) * p2) + (pow(t, 3) * p3)

    return formula

def drawdebugSquares(pts,heading):
    for pt in pts:
        pt = np.array(pt)
        r = 0.01
        #print(pt+[r,r,0] )
        rot = np.array([[np.cos(heading), -np.sin(heading)],
                   [np.sin(heading), np.cos(heading)]])
        FR = np.matmul(rot,np.array([r,r]))
        FL = np.matmul(rot,np.array([-r,r]))
        RR = np.matmul(rot,np.array([r,-r]))
        RL = np.matmul(rot,np.array([-r,-r]))
        FR = np.concatenate((FR,np.zeros(1)))
        FL = np.concatenate((FL,np.zeros(1)))
        RR = np.concatenate((RR,np.zeros(1)))
        RL = np.concatenate((RL,np.zeros(1)))

        p.addUserDebugLine(pt+FR ,pt+FL , [0,0,1])
        p.addUserDebugLine(pt+FL ,pt+RL, [0,0,1])
        p.addUserDebugLine(pt+RL ,pt+RR , [0,0,1])
        p.addUserDebugLine(pt+RR ,pt+FR  , [0,0,1] )
    return

def getStepPositions(pos,heading,step):
    pos = np.array([pos[0],pos[1]])
    rot = np.array([[np.cos(heading), -np.sin(heading)],
                   [np.sin(heading), np.cos(heading)]])

    forwardstep = .1
    backstep = .1
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
h = 0.3
dogStartPos = [0, 0, h]
dogStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
dogId = p.loadURDF("model/robot.urdf", [0, 0, 0.45], dogStartOrientation)


# dogId = p.loadURDF("model/robot.urdf", [0, 0, 0.45], dogStartOrientation)
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
p0 = [0, 0, h]
p1 = [2, -1, h]
p2 = [4, 2, h]
p3 = [6, -3, h]

p1Id = p.loadURDF("model/testcube.urdf", p1)
p2Id = p.loadURDF("model/testcube.urdf", p2)
p3Id = p.loadURDF("model/testcube.urdf", p3)

# Create Stepping Stones
sFL = np.array([0.3,0.15,1e-4])
sFR = np.array([0.3,-0.15,1e-4])
sRL = np.array([-0.1,0.15,1e-4])
sRR = np.array([-0.1,-0.15,1e-4])

bzPoints = [p0, p1, p2, p3]

bezier = createBezier(bzPoints)

t = 0

# Dog Gait Settings
path_period = 1550  # Period that path takes
gait_length = 0.15 # Largest step size
body_height = 0.3 # Height of hips above ground

# Camera Settings
cyaw=0
cpitch=-90
cdist=5

pos, orientation = p.getBasePositionAndOrientation(dogId)

if(TOPDOWN):
    cpitch = -89
p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=pos)

currentPoint = p1Id

closeup = False
if(closeup):
    p.resetDebugVisualizerCamera( cameraDistance=.75, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=dogStartPos)

forced = True

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
    
    # Reset and Visualize Path
    if keys.get(ord('c')) and keys[ord('c')]&p.KEY_WAS_TRIGGERED:
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(dogId, [0,0,0.45], dogStartOrientation)
        p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=DEFAULT_JOINT_POS)
        sFL = np.array([0.3,0.15,1e-4])
        sFR = np.array([0.3,-0.15,1e-4])
        sRL = np.array([-0.1,0.15,1e-4])
        sRR = np.array([-0.1,-0.15,1e-4])
        drawdebugSquares([sFL, sFR, sRL, sRR],0)
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
            if (linecount % 3000) == 0:
                p.addUserDebugLine(lastPos,targetPos,[1,0,0]) 
                lastPos = targetPos
            targetPos = bezier(t)
            yaw = math.atan2(targetPos[1]-lastPos[1], targetPos[0]-lastPos[0])
            if (linecount % math.floor(path_period)) == 0:
                FR, FL, RR, RL, sFR, sFL, sRR, sRL = getStepPositions(targetPos,yaw,gait_length)
                drawdebugSquares([sFR, sFL, sRR, sRL],yaw)

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
    if keys.get(ord('i')) and keys[ord('i')]&p.KEY_WAS_TRIGGERED:  
        forced = not forced


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
        error = 0
        p.resetBasePositionAndOrientation(dogId, dogStartPos, dogStartOrientation)
        #drawdebugSquares([sFL, sFR, sRL, sRR])
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
            
            #p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=targetPos)
            dogPos, _ = p.getBasePositionAndOrientation(dogId)
            p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=dogPos)
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

            # Move feet
            ## TODO: sample footpath with timestep to get desired XYs

            # t0 = 0.5
            t0 = count / path_period
            t1 = t0 + 0.5 # assuming that the swing and contact portions of the gait are exactly the same length!!

            x0, y0 = foot_path(t = t0, length=gait_length, body_height=body_height)
            x1, y1 = foot_path(t = t1, length=gait_length, body_height=body_height)

            ## convert the desired xy positions into angles
            # theta, phi = shoulder, wrist
            theta0, phi0 = leg_ik(x0, y0, 0.2, 0.2)
            theta1, phi1 = leg_ik(x1, y1, 0.2, 0.2)
            
            ## TODO: Pass computed IK angles into pybullet sim 
            pair0 = [0, theta0, phi0]
            pair1 = [0, theta1, phi1]

            stepAngles = np.array([pair0, pair1, pair1, pair0])
            stepAngles = stepAngles.flatten()

            #stepAngles = p.calculateInverseKinematics2(dogId, [5,10,15,20], [sFR,FL,RR,sRL])

            p.setJointMotorControlArray(dogId,JOINTS,controlMode=p.POSITION_CONTROL,targetPositions=stepAngles)

            # FORCED vs FREE MOTION
            if forced:
                p.resetBasePositionAndOrientation(dogId, targetPos, newOrientation)

            p.stepSimulation()
            count += 1
            current, _ = p.getBasePositionAndOrientation(dogId)
            error = current-targetPos
        p.addUserDebugLine(lastPos,targetPos,[1,0,0]) 
        for t in np.arange(0, 1, .005):
            p.stepSimulation()
    
    
    time.sleep(1./240.)
