import pybullet as p 
import pybullet_data 
import time
from PID import PID 

p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.loadURDF("plane.urdf") 
p.setGravity(0,0,-9.81)

robot_id = p.loadURDF("franka_panda/panda.urdf",[0,0,0], useFixedBase=True) 

print("Franka Panda Loaded into simulation")

#360 view
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) 
p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0]) 

#Rendering the colours

p.changeVisualShape(robot_id, 1, rgbaColor=[1, 0, 0, 1]) 
p.changeVisualShape(robot_id, 2, rgbaColor=[0, 0, 1, 1])
p.changeVisualShape(robot_id, 3, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(robot_id, 4, rgbaColor=[1, 1, 0, 1])

#Collision detection
p.setCollisionFilterPair(robot_id,robot_id, 1, 2, enableCollision=True)   
p.setCollisionFilterPair(robot_id,robot_id, 1, 3, enableCollision=True)
p.setCollisionFilterPair(robot_id,robot_id, 1, 4, enableCollision=True)
p.setCollisionFilterPair(robot_id,robot_id, 2, 3, enableCollision=True)
p.setCollisionFilterPair(robot_id,robot_id, 2, 4, enableCollision=True)
p.setCollisionFilterPair(robot_id,robot_id, 3, 4, enableCollision=True)

#PID Setup
pids = [
    PID(Kp=30, Ki=1, Kd=15),
    PID(Kp=30, Ki=1, Kd=15),
    PID(Kp=30, Ki=1, Kd=15),
    PID(Kp=30, Ki=1, Kd=15) 
]   



targets = [1.0, -0.5, 0.5, 0.0]


dt = 1/240

#joint index
num_joints = p.getNumJoints(robot_id)
joint_index = 0 

print("Simulation is running")
print("Use left click to move the camera and right click to rotate the camera")
print("To zoom in scroll")
print("Click close to exit the simulation")

#Simulation loop
while p.isConnected():
    try:
        for i in range(4):
            joint_state = p.getJointState(robot_id, i)
            current_position = joint_state[0]
            
        
        # Calculate PID output
        force = pids[i].update(targets[i], current_position, dt)
        
      
        p.setJointMotorControl2(robot_id,joint_index,p.TORQUE_CONTROL,force=force)
        
        p.stepSimulation()
        time.sleep(dt)
    except:
        break  

p.disconnect()
print("Simulation closed.")
